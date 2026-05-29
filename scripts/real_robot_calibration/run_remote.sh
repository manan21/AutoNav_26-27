#!/usr/bin/env bash
# Laptop entrypoint. Sync this suite to the robot and run a profile over SSH.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILES_FILE="$SCRIPT_DIR/profiles.yaml"

usage() {
  cat >&2 <<'EOF'
Usage:
  run_remote.sh PROFILE [options]

Options:
  --robot HOST              SSH target, default: jetson
  --base-dir DIR            Robot bag root, default: /tmp/autonav_bags/practice_course
  --remote-suite-dir DIR    Robot staging dir, default: ~/AutoNav_25-26/scripts/real_robot_calibration
  --session NAME            tmux session name, default: autonav_calib_bag
  --run-name NAME           Override timestamped run name on the robot
  --allow-high-speed        Permit profiles marked high-speed
  --raw-lidar               Add /cloud_all_fields_fullframe to the bag
  --no-tmux                 Run directly over SSH instead of inside tmux
  --dry-run                 Print what would run; do not SSH or move robot
  --list                    List profiles

Examples:
  ./run_remote.sh record_manual_full_course
  ./run_remote.sh straight_speed_ladder_low
  ./run_remote.sh straight_speed_ladder_high --allow-high-speed
EOF
}

if [ $# -eq 0 ]; then
  usage
  exit 2
fi

if [ "$1" = "--list" ]; then
  python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --list
  exit 0
fi

PROFILE=$1
shift

ROBOT=${AUTONAV_CALIB_ROBOT:-jetson}
BASE_DIR=${AUTONAV_CALIB_BASE_DIR:-}
REMOTE_SUITE_DIR=${AUTONAV_CALIB_REMOTE_SUITE_DIR:-}
SESSION=${AUTONAV_CALIB_TMUX_SESSION:-autonav_calib_bag}
RUN_NAME=""
if [ -z "$BASE_DIR" ]; then
  BASE_DIR='/tmp/autonav_bags/practice_course'
fi
if [ -z "$REMOTE_SUITE_DIR" ]; then
  REMOTE_SUITE_DIR='~/AutoNav_25-26/scripts/real_robot_calibration'
fi
ALLOW_HIGH_SPEED=0
RAW_LIDAR=0
USE_TMUX=1
DRY_RUN=0

while [ $# -gt 0 ]; do
  case "$1" in
    --robot)
      ROBOT=${2:?missing --robot value}
      shift 2
      ;;
    --base-dir)
      BASE_DIR=${2:?missing --base-dir value}
      shift 2
      ;;
    --remote-suite-dir)
      REMOTE_SUITE_DIR=${2:?missing --remote-suite-dir value}
      shift 2
      ;;
    --session)
      SESSION=${2:?missing --session value}
      shift 2
      ;;
    --run-name)
      RUN_NAME=${2:?missing --run-name value}
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
    --no-tmux)
      USE_TMUX=0
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
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

python3 "$SCRIPT_DIR/profile_info.py" --profiles "$PROFILES_FILE" --profile "$PROFILE" --field description >/dev/null

q() {
  printf "%q" "$1"
}

LOCAL_GIT_BRANCH=$(git -C "$SCRIPT_DIR" branch --show-current 2>/dev/null || true)
LOCAL_GIT_COMMIT=$(git -C "$SCRIPT_DIR" rev-parse --short=12 HEAD 2>/dev/null || true)

RUN_ON_ROBOT_ARGS="$(q "$PROFILE") --base-dir $(q "$BASE_DIR")"
if [ "$ALLOW_HIGH_SPEED" -eq 1 ]; then
  RUN_ON_ROBOT_ARGS+=" --allow-high-speed"
fi
if [ "$RAW_LIDAR" -eq 1 ]; then
  RUN_ON_ROBOT_ARGS+=" --raw-lidar"
fi
if [ -n "$RUN_NAME" ]; then
  RUN_ON_ROBOT_ARGS+=" --run-name $(q "$RUN_NAME")"
fi
REMOTE_RUN_CMD="if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx koopa-kingdom; then docker exec -it -u admin -e AUTONAV_CALIB_GIT_BRANCH=$(q "${LOCAL_GIT_BRANCH:-unknown}") -e AUTONAV_CALIB_GIT_COMMIT=$(q "${LOCAL_GIT_COMMIT:-unknown}") koopa-kingdom bash -lc $(q "cd /autonav/scripts/real_robot_calibration && ./run_on_robot.sh $RUN_ON_ROBOT_ARGS"); else cd $(q "$REMOTE_SUITE_DIR") && AUTONAV_CALIB_GIT_BRANCH=$(q "${LOCAL_GIT_BRANCH:-unknown}") AUTONAV_CALIB_GIT_COMMIT=$(q "${LOCAL_GIT_COMMIT:-unknown}") ./run_on_robot.sh $RUN_ON_ROBOT_ARGS; fi"

echo "Profile:"
DRY_RUN_ARGS=(--profiles "$PROFILES_FILE" --profile "$PROFILE" --dry-run)
if [ "$ALLOW_HIGH_SPEED" -eq 1 ]; then
  DRY_RUN_ARGS+=(--allow-high-speed)
fi
python3 "$SCRIPT_DIR/cmd_profile_runner.py" "${DRY_RUN_ARGS[@]}"
echo
echo "Robot: $ROBOT"
echo "Remote suite dir: $REMOTE_SUITE_DIR"
echo "Remote bag root: $BASE_DIR"

if [ "$DRY_RUN" -eq 1 ]; then
  echo
  echo "Dry run. Would sync $SCRIPT_DIR and execute:"
  echo "  ssh $ROBOT $(q "$REMOTE_RUN_CMD")"
  exit 0
fi

echo
echo "Syncing calibration suite to $ROBOT:$REMOTE_SUITE_DIR ..."
REMOTE_SUITE_Q=$(q "$REMOTE_SUITE_DIR")
env COPYFILE_DISABLE=1 LC_ALL=C LANG=C tar -C "$SCRIPT_DIR" --exclude='__pycache__' --exclude='*.pyc' --exclude='.DS_Store' --exclude='._*' -czf - . \
  | ssh "$ROBOT" "rm -rf $REMOTE_SUITE_Q && mkdir -p $REMOTE_SUITE_Q && env LC_ALL=C LANG=C tar -xzf - -C $REMOTE_SUITE_Q"

if [ "$USE_TMUX" -eq 1 ]; then
  echo "Starting tmux session '$SESSION' on $ROBOT. Detach with Ctrl-b d; stop with ./stop_remote.sh."
  TMUX_CMD="tmux new-session -s $(q "$SESSION") $(q "$REMOTE_RUN_CMD; echo; echo 'Calibration run finished. Press Enter to close this shell, or Ctrl-b d to leave tmux open.'; exec bash")"
  ssh -tt "$ROBOT" "$TMUX_CMD"
else
  ssh -tt "$ROBOT" "$REMOTE_RUN_CMD"
fi
