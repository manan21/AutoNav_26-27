#!/usr/bin/env bash
# Jetson-side recorder/commander. Usually invoked by run_remote.sh.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROFILES_FILE="$SCRIPT_DIR/profiles.yaml"
STATE_DIR=${AUTONAV_CALIB_STATE_DIR:-$HOME/.autonav_real_robot_calibration}
ACTIVE_FILE="$STATE_DIR/active_run.env"

usage() {
  cat >&2 <<'EOF'
Usage:
  run_on_robot.sh PROFILE [options]

Options:
  --base-dir DIR            Bag root, default: persistent auto-selected path
  --allow-high-speed        Permit profiles marked high-speed
  --raw-lidar               Add /cloud_all_fields_fullframe
  --run-name NAME           Override timestamped run name
EOF
}

default_base_dir() {
  # Preferred container mount from env/docker/run-container.sh. This keeps bags
  # outside the git checkout on the Jetson host.
  if [ -d /autonav_bags ] && [ -w /autonav_bags ]; then
    printf '%s\n' "/autonav_bags/practice_course"
    return
  fi

  # Backward-compatible fallback for already-running robot containers. /autonav
  # is the host AutoNav_25-26 checkout bind-mounted into koopa-kingdom, so this
  # survives power cycles, branch switches, and `git reset --hard`.
  if [ -d /autonav ] && [ -w /autonav ]; then
    printf '%s\n' "/autonav/logs/real_robot_calibration"
    return
  fi

  # Native Jetson execution path. This is intentionally outside AutoNav_25-26.
  printf '%s\n' "$HOME/autonav_bags/practice_course"
}

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    local status=$?
    set -u
    return "$status"
  fi
}

publish_zero() {
  if command -v ros2 >/dev/null 2>&1; then
    timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
      >/dev/null 2>&1 || true
  fi
}

if [ $# -eq 0 ]; then
  usage
  exit 2
fi

PROFILE=$1
shift

BASE_DIR=${AUTONAV_CALIB_BASE_DIR:-$(default_base_dir)}
ALLOW_HIGH_SPEED=0
RAW_LIDAR=0
RUN_NAME=""

while [ $# -gt 0 ]; do
  case "$1" in
    --base-dir)
      BASE_DIR=${2:?missing --base-dir value}
      shift 2
      ;;
    --allow-high-speed)
      ALLOW_HIGH_SPEED=1
      shift
      ;;
    --raw-lidar)
      RAW_LIDAR=1
      shift
      ;;
    --run-name)
      RUN_NAME=${2:?missing --run-name value}
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

# Expand a leading ~ after argument parsing.
BASE_DIR="${BASE_DIR/#\~/$HOME}"

source_setup /opt/ros/humble/setup.bash
source_setup "$REPO_ROOT/isaac_ros-dev/install/setup.bash"
source_setup "/autonav/isaac_ros-dev/install/setup.bash"
source_setup "$HOME/AutoNav_25-26/isaac_ros-dev/install/setup.bash"
source_setup "$HOME/code/git/AutoNav_25-26/isaac_ros-dev/install/setup.bash"
source_setup "/workspaces/isaac_ros-dev/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found. Source the robot ROS environment before running this script." >&2
  exit 1
fi

if ! ros2 interface show autonav_interfaces/msg/Encoders >/dev/null 2>&1; then
  echo "autonav_interfaces/msg/Encoders is not available in the ROS environment." >&2
  echo "Source the AutoNav workspace install before recording; otherwise /encoders cannot be recorded." >&2
  exit 1
fi

python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field description >/dev/null
REQUIRES_HIGH=$(python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field requires_allow_high_speed)
if [ "$REQUIRES_HIGH" = "true" ] && [ "$ALLOW_HIGH_SPEED" -ne 1 ]; then
  echo "Profile '$PROFILE' requires --allow-high-speed." >&2
  exit 2
fi

BAG_PROFILE=$(python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field bag_profile)
COMMAND_MODE=$(python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field command_mode)
RECORD_UNTIL_INTERRUPT=$(python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field record_until_interrupt)
STRICT_REQUIRED=$(python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field strict_required_topics)
TOPIC_FILE="$SCRIPT_DIR/topics/$BAG_PROFILE.txt"
REQUIRED_TOPIC_FILE="$SCRIPT_DIR/required_topics/$BAG_PROFILE.txt"

if [ ! -f "$TOPIC_FILE" ]; then
  echo "Topic file not found: $TOPIC_FILE" >&2
  exit 1
fi

if [ -z "$RUN_NAME" ]; then
  RUN_NAME="${PROFILE}_$(date +%Y%m%d_%H%M%S)"
fi

RUN_DIR="$BASE_DIR/$RUN_NAME"
BAG_PATH="$RUN_DIR/bag"
BAG_LOG="$RUN_DIR/rosbag_record.log"
COMMAND_LOG="$RUN_DIR/command_profile.log"
TOPIC_SNAPSHOT="$RUN_DIR/topic_list_at_start.txt"
MISSING_TOPICS="$RUN_DIR/missing_topics_at_start.txt"
MISSING_REQUIRED_TOPICS="$RUN_DIR/missing_required_topics_at_start.txt"
METADATA="$RUN_DIR/run_metadata.txt"

mkdir -p "$RUN_DIR" "$STATE_DIR"
cp "$PROFILES_FILE" "$RUN_DIR/profiles.yaml"
cp "$TOPIC_FILE" "$RUN_DIR/topics.txt"
if [ -f "$REQUIRED_TOPIC_FILE" ]; then
  cp "$REQUIRED_TOPIC_FILE" "$RUN_DIR/required_topics.txt"
fi

mapfile -t TOPICS < <(grep -Ev '^[[:space:]]*(#|$)' "$TOPIC_FILE")
if [ "$RAW_LIDAR" -eq 1 ]; then
  TOPICS+=("/cloud_all_fields_fullframe")
fi

METADATA_ARGS=(
  --profiles "$PROFILES_FILE" \
  --profile "$PROFILE" \
  --write-metadata "$METADATA" \
  --run-name "$RUN_NAME" \
  --bag-path "$BAG_PATH" \
  --topic-file "$TOPIC_FILE" \
)
if [ "$ALLOW_HIGH_SPEED" -eq 1 ]; then
  METADATA_ARGS+=(--allow-high-speed)
fi
if [ "$RAW_LIDAR" -eq 1 ]; then
  METADATA_ARGS+=(--raw-lidar)
fi
METADATA_ARGS+=(--argv "$0" "$PROFILE" "$@")
python3 "$SCRIPT_DIR/profile_info.py" "${METADATA_ARGS[@]}"

ros2 topic list >"$TOPIC_SNAPSHOT" 2>/dev/null || true
: >"$MISSING_TOPICS"
for topic in "${TOPICS[@]}"; do
  if ! grep -Fx "$topic" "$TOPIC_SNAPSHOT" >/dev/null 2>&1; then
    echo "$topic" >>"$MISSING_TOPICS"
  fi
done
: >"$MISSING_REQUIRED_TOPICS"
if [ -f "$REQUIRED_TOPIC_FILE" ]; then
  while IFS= read -r topic; do
    topic="${topic%%#*}"
    topic="$(echo "$topic" | xargs)"
    [ -z "$topic" ] && continue
    if ! grep -Fx "$topic" "$TOPIC_SNAPSHOT" >/dev/null 2>&1; then
      echo "$topic" >>"$MISSING_REQUIRED_TOPICS"
    fi
  done <"$REQUIRED_TOPIC_FILE"
fi

echo "Run name: $RUN_NAME"
echo "Profile: $PROFILE"
echo "Bag profile: $BAG_PROFILE"
echo "Strict required topics: $STRICT_REQUIRED"
echo "Bag path: $BAG_PATH"
echo "Metadata: $METADATA"
echo
echo "VIDEO SYNC:"
echo "  1. Say: $RUN_NAME"
echo "  2. Say the current phone/video time."
echo "  3. Wave in front of the camera and lidar."
echo
if [ -s "$MISSING_TOPICS" ]; then
  echo "Topics not visible at recorder start; rosbag may still capture them if they appear later:"
  sed 's/^/  /' "$MISSING_TOPICS"
  echo
fi
if [ -s "$MISSING_REQUIRED_TOPICS" ]; then
  echo "Required topics missing at recorder start:"
  sed 's/^/  /' "$MISSING_REQUIRED_TOPICS"
  echo
  if [ "$STRICT_REQUIRED" = "true" ]; then
    echo "Strict required-topic preflight failed; not starting rosbag." >&2
    echo "Run directory: $RUN_DIR" >&2
    exit 3
  fi
fi

echo "Disk status for bag root:"
df -h "$BASE_DIR" || true
echo

BAG_PID=""
COMMAND_STATUS=0
CLEANED_UP=0

write_active_state() {
  {
    printf 'RUNNER_PID=%q\n' "$$"
    printf 'RUN_DIR=%q\n' "$RUN_DIR"
    printf 'BAG_PATH=%q\n' "$BAG_PATH"
    printf 'BAG_PID=%q\n' "${BAG_PID:-}"
  } >"$ACTIVE_FILE"
}

cleanup() {
  local exit_status=$?
  if [ "$CLEANED_UP" -eq 1 ]; then
    exit "$exit_status"
  fi
  CLEANED_UP=1
  echo
  echo "Stopping robot commands and bag recorder..."
  publish_zero
  if [ -n "${BAG_PID:-}" ] && kill -0 "$BAG_PID" >/dev/null 2>&1; then
    kill -INT "$BAG_PID" >/dev/null 2>&1 || true
    wait "$BAG_PID" >/dev/null 2>&1 || true
  fi
  publish_zero
  rm -f "$ACTIVE_FILE"
  if [ -d "$BAG_PATH" ]; then
    VERIFY_ARGS=("$BAG_PATH")
    if [ -f "$REQUIRED_TOPIC_FILE" ]; then
      VERIFY_ARGS+=(--required-topic-file "$REQUIRED_TOPIC_FILE")
      if [ "$STRICT_REQUIRED" = "true" ]; then
        VERIFY_ARGS+=(--strict)
      fi
    fi
    if ! "$SCRIPT_DIR/verify_bag.sh" "${VERIFY_ARGS[@]}"; then
      if [ "$STRICT_REQUIRED" = "true" ] && [ "$exit_status" -eq 0 ]; then
        exit_status=1
      fi
    fi
  fi
  echo "Run directory: $RUN_DIR"
  exit "$exit_status"
}
trap cleanup EXIT INT TERM

echo "Starting rosbag recorder..."
ros2 bag record --include-hidden-topics \
  --max-cache-size 1000000000 \
  -o "$BAG_PATH" \
  "${TOPICS[@]}" \
  >"$BAG_LOG" 2>&1 &
BAG_PID=$!
write_active_state

sleep 3
if ! kill -0 "$BAG_PID" >/dev/null 2>&1; then
  echo "ros2 bag record exited early. See $BAG_LOG" >&2
  exit 1
fi

echo "Recorder is running. Log: $BAG_LOG"
echo

if [ "$COMMAND_MODE" = "scripted" ]; then
  echo "Starting scripted /cmd_vel profile. Toggle AUTO with Xbox X only when safe."
  set +e
  COMMAND_ARGS=(
    --profiles "$PROFILES_FILE" \
    --profile "$PROFILE" \
    --metrics-file "$RUN_DIR/command_metrics.csv" \
  )
  if [ "$ALLOW_HIGH_SPEED" -eq 1 ]; then
    COMMAND_ARGS+=(--allow-high-speed)
  fi
  python3 "$SCRIPT_DIR/cmd_profile_runner.py" "${COMMAND_ARGS[@]}" 2>&1 | tee "$COMMAND_LOG"
  COMMAND_STATUS=${PIPESTATUS[0]}
  set -e
  echo "Scripted command status: $COMMAND_STATUS"
  exit "$COMMAND_STATUS"
elif [ "$RECORD_UNTIL_INTERRUPT" = "true" ]; then
  echo "No scripted robot command for this profile."
  echo "Recording until Ctrl-C. For manual profiles, drive now. For closed-loop profiles, use RViz/Nav2 now."
  while true; do
    sleep 1
  done
else
  echo "No command and not record_until_interrupt; holding bag for 10 seconds."
  sleep 10
fi
