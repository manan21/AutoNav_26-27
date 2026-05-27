#!/bin/bash
# Run the measured lidar-line avoidance test.
#
# This script intentionally does not toggle autonomous mode. The human
# operator must put the robot in autonomous mode before running it.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source_setup() {
  local setup_file=$1

  if [ -f "$setup_file" ]; then
    # ROS setup files can reference unset tracing variables.
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    local source_status=$?
    set -u
    return "$source_status"
  fi
}

source_setup /opt/ros/humble/setup.bash
source_setup "$ISAAC_ROOT/install/setup.bash"

RUN_NAME="${1:-lidar_line_test_$(date +%Y%m%d_%H%M%S)}"
LOG_ROOT="${LIDAR_LINE_TEST_LOG_ROOT:-/autonav/logs}"
RUN_DIR="$LOG_ROOT/$RUN_NAME"
BAG_PATH="$RUN_DIR/bag"
GOAL_LOG="$RUN_DIR/navigate_to_pose.log"
BAG_LOG="$RUN_DIR/rosbag_record.log"

TOPICS=(
  /tf
  /tf_static
  /local_ekf/odom
  /odom
  /cmd_vel
  /cmd_vel_nav
  /encoders
  /motor_speed
  /autonomous_mode
  /scan_fullframe
  /scan_pca_filtered
  /scan_pca_filtered_clear
  /lidar_line_points
  /lidar_line_costmap
  /lidar_line_detection/diagnostics
  /scan_pca_filtered_points
  /terrain/grade_map
  /pca/surface_normal
  /local_costmap/costmap
  /local_costmap/costmap_raw
  /global_costmap/costmap
  /global_costmap/costmap_raw
  /plan
  /local_plan
  /goal_pose
  /evaluation
  /navigate_to_pose/_action/status
  /follow_path/_action/status
  /compute_path_to_pose/_action/status
)

mkdir -p "$RUN_DIR"

echo "Lidar-line test run: $RUN_NAME"
echo "Bag path: $BAG_PATH"
echo "Human operator must already have autonomous mode enabled."

echo "Clearing stale local/global costmap memory..."
timeout 5 ros2 topic pub --once /local_mirror_layer/clear std_msgs/msg/Empty "{}" >/dev/null 2>&1 || true
timeout 10 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null
timeout 10 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null

echo "Starting rosbag recorder..."
ros2 bag record --include-hidden-topics -o "$BAG_PATH" "${TOPICS[@]}" >"$BAG_LOG" 2>&1 &
BAG_PID=$!

cleanup() {
  if kill -0 "$BAG_PID" >/dev/null 2>&1; then
    echo "Stopping rosbag recorder..."
    kill -INT "$BAG_PID" >/dev/null 2>&1 || true
    wait "$BAG_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

sleep 3
if ! kill -0 "$BAG_PID" >/dev/null 2>&1; then
  echo "rosbag recorder exited early; see $BAG_LOG" >&2
  exit 1
fi

echo "Sending one direct NavigateToPose goal: 2.0 m relative forward."
set +e
"$SCRIPT_DIR/send_goal.sh" --action -r 2.0 0 0 2>&1 | tee "$GOAL_LOG"
GOAL_STATUS=${PIPESTATUS[0]}
set -e

sleep 2
cleanup
trap - EXIT INT TERM

echo
echo "Goal command exit status: $GOAL_STATUS"
echo "Action log: $GOAL_LOG"
echo "Bag log: $BAG_LOG"
echo "Bag path: $BAG_PATH"
echo
echo "Recommended analysis:"
echo "  python3 scripts/analyze_lidar_line_bag.py $BAG_PATH"
echo "  python3 scripts/analyze_dwb_evaluation.py $BAG_PATH --window 0.1"
echo "  python3 scripts/analyze_costmap_footprint.py $BAG_PATH --hard-threshold 100"

exit "$GOAL_STATUS"
