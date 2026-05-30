#!/usr/bin/env bash
# One headless IGVC sim run for the build-loop. Replicates the proven launch +
# setsid process-group cleanup from Run_IGVC_COMPETITION_FORTRESS_TEST.command,
# but (a) passes world:= and SOURCE nav2_params/bt_xml so YAML/BT edits need no
# rebuild, and (b) records a metric-bearing superset bag that includes the
# recovery signals (/rosout + behavior action statuses) the stock test omits.
#
# Writes RUN_DIR/{bag/, final_score.txt, mission.log}. Always cleans up its
# process group + runs the reaper. Run ON the machine that has the sim stack.
#
# UNVALIDATED against a live sim (none runnable on this host yet); structurally
# mirrors the verified .command. See references/CONTEXT.md.
#
# Usage:
#   run_one.sh --course-yaml <yaml> --world <sdf> --run-dir <dir> [--timeout 300]
# Env overrides:
#   ROS_WS (default: repo isaac_ros-dev), NAV2_PARAMS_SRC, BT_XML_SRC,
#   STARTUP_WAIT_SEC (12), PRE_MISSION_WAIT_SEC (8), ROS_DOMAIN_ID (auto)
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../../../.." && pwd)"   # .../AutoNav_25-26
ROS_WS="${ROS_WS:-$REPO/isaac_ros-dev}"
SRC="$ROS_WS/src"
NAV2_PARAMS_SRC="${NAV2_PARAMS_SRC:-$SRC/slam/config/nav2_params_camera.yaml}"
BT_XML_SRC="${BT_XML_SRC:-$SRC/slam/behavior_trees/bt_nav.xml}"
DYN_CAL="${DYNAMICS_CALIBRATION:-$SRC/igvc_competition_sim/config/dynamics_calibration.yaml}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-12}"
PRE_MISSION_WAIT_SEC="${PRE_MISSION_WAIT_SEC:-8}"
MISSION_TIMEOUT_SEC="${MISSION_TIMEOUT_SEC:-300}"

COURSE_YAML=""; WORLD=""; RUN_DIR=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --course-yaml) COURSE_YAML="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --run-dir) RUN_DIR="$2"; shift 2;;
    --timeout) MISSION_TIMEOUT_SEC="$2"; shift 2;;
    *) echo "unknown arg $1" >&2; exit 2;;
  esac
done
[[ -z "$COURSE_YAML" || -z "$WORLD" || -z "$RUN_DIR" ]] && {
  echo "usage: run_one.sh --course-yaml X --world Y --run-dir Z [--timeout N]" >&2; exit 2; }
for f in "$COURSE_YAML" "$WORLD" "$NAV2_PARAMS_SRC" "$BT_XML_SRC"; do
  [[ -f "$f" ]] || { echo "missing required file: $f" >&2; exit 2; }
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( (RANDOM % 200) + 11 ))}"
mkdir -p "$RUN_DIR"

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "ROS env not ready (need /opt/ros/humble + $ROS_WS/install). Build the workspace." >&2
  exit 3
fi
set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

stop_process() {  # pid name [group]
  local pid="$1" name="$2" scope="${3:-process}" target="$1"
  [[ "$scope" == "group" ]] && target="-$pid"
  is_running() { [[ "$scope" == "group" ]] && pgrep -g "$pid" >/dev/null 2>&1 || kill -0 "$pid" 2>/dev/null; }
  [[ -z "$pid" ]] || ! is_running && return 0
  kill -INT -- "$target" 2>/dev/null || true
  for _ in {1..20}; do is_running || { wait "$pid" 2>/dev/null||true; return 0; }; sleep 0.25; done
  kill -TERM -- "$target" 2>/dev/null || true
  for _ in {1..20}; do is_running || { wait "$pid" 2>/dev/null||true; return 0; }; sleep 0.25; done
  kill -KILL -- "$target" 2>/dev/null || true; wait "$pid" 2>/dev/null || true
}
cleanup() {
  set +e
  [[ -n "${bag_pid:-}" ]] && stop_process "$bag_pid" "rosbag recorder"
  [[ -n "${stack_pid:-}" ]] && stop_process "$stack_pid" "IGVC stack" "group"
  bash "$HERE/reaper.sh" >>"$RUN_DIR/reaper.log" 2>&1 || true
}
trap cleanup EXIT INT TERM

# fresh slate
bash "$HERE/reaper.sh" >>"$RUN_DIR/reaper.log" 2>&1 || true

setsid ros2 launch igvc_competition_sim igvc_competition.launch.py \
  course_config:="$COURSE_YAML" \
  world:="$WORLD" \
  line_detection_mode:=camera \
  nav2_params:="$NAV2_PARAMS_SRC" \
  bt_xml:="$BT_XML_SRC" \
  use_calibrated_dynamics:=true \
  dynamics_calibration:="$DYN_CAL" \
  gazebo_server_only:=true \
  >"$RUN_DIR/launch.log" 2>&1 &
stack_pid=$!

sleep "$STARTUP_WAIT_SEC"

ros2 bag record -o "$RUN_DIR/bag" \
  /clock /tf /tf_static \
  /odom /local_ekf/odom /igvc_sim/ground_truth_odom \
  /cmd_vel /cmd_vel_nav \
  /navigate_to_pose/_action/status /navigate_to_waypoint/_action/status \
  /follow_path/_action/status /compute_path_to_pose/_action/status \
  /back_up/_action/status /spin/_action/status /drive_on_heading/_action/status \
  /rosout \
  /plan /unsmoothed_plan \
  /global_costmap/costmap_raw /local_costmap/costmap_raw \
  /line_points /line_costmap /scan_pca_filtered \
  /igvc_sim/score /igvc_sim/fail \
  >"$RUN_DIR/bag_record.log" 2>&1 &
bag_pid=$!

sleep "$PRE_MISSION_WAIT_SEC"

mission_status=0
set +e
timeout --foreground "${MISSION_TIMEOUT_SEC}s" \
  ros2 run igvc_competition_sim igvc_mission_runner \
    --course-config "$COURSE_YAML" --timeout-sec "$MISSION_TIMEOUT_SEC" \
  | tee "$RUN_DIR/mission.log"
mission_status="${PIPESTATUS[0]}"
ros2 topic echo --once /igvc_sim/score > "$RUN_DIR/final_score.txt" 2>&1
set -e

echo "$mission_status" > "$RUN_DIR/mission_status.txt"
sleep 2
cleanup
trap - EXIT INT TERM
echo "run_one: done (mission_status=$mission_status) -> $RUN_DIR"
exit 0
