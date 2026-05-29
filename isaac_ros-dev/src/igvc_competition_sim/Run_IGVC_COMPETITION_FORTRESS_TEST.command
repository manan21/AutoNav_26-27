#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTONAV_REPO="${AUTONAV_REPO:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
ROS_WS="${ROS_WS:-$AUTONAV_REPO/isaac_ros-dev}"
COURSE_CONFIG="${COURSE_CONFIG:-$SCRIPT_DIR/config/igvc_competition_compact.yaml}"
RUN_DIR="${RUN_DIR:-$SCRIPT_DIR/fortress_runs/$(basename "$COURSE_CONFIG" .yaml)_$(date +%Y%m%d_%H%M%S)}"
MISSION_TIMEOUT_SEC="${MISSION_TIMEOUT_SEC:-300}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-12}"
PRE_MISSION_WAIT_SEC="${PRE_MISSION_WAIT_SEC:-8}"
GROUND_TRUTH_PCA="${GROUND_TRUTH_PCA:-false}"
LINE_DETECTION_MODE="${LINE_DETECTION_MODE:-camera}"
LAUNCH_GAZEBO="${LAUNCH_GAZEBO:-true}"
GAZEBO_SERVER_ONLY="${GAZEBO_SERVER_ONLY:-true}"
LAUNCH_BRIDGE="${LAUNCH_BRIDGE:-true}"
if [[ -z "${NAV2_PARAMS:-}" ]]; then
  if [[ "$LINE_DETECTION_MODE" == "lidar" ]]; then
    NAV2_PARAMS="$ROS_WS/install/slam/share/slam/config/nav2_params_lidar.yaml"
  else
    NAV2_PARAMS="$ROS_WS/install/slam/share/slam/config/nav2_params_camera.yaml"
  fi
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS Humble setup not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "AutoNav workspace install setup not found:" >&2
  echo "  $ROS_WS/install/setup.bash" >&2
  echo "Build the workspace first: cd $ROS_WS && colcon build" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

mkdir -p "$RUN_DIR"

stop_process() {
  local pid="$1"
  local name="$2"
  local scope="${3:-process}"
  local target="$pid"
  if [[ "$scope" == "group" ]]; then
    target="-$pid"
  fi
  is_running() {
    if [[ "$scope" == "group" ]]; then
      pgrep -g "$pid" >/dev/null 2>&1
    else
      kill -0 "$pid" 2>/dev/null
    fi
  }
  if [[ -z "$pid" ]] || ! is_running; then
    return
  fi
  kill -INT -- "$target" 2>/dev/null || true
  for _ in {1..20}; do
    if ! is_running; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.25
  done
  echo "Timed out waiting for $name to stop; sending SIGTERM." >&2
  kill -TERM -- "$target" 2>/dev/null || true
  for _ in {1..20}; do
    if ! is_running; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.25
  done
  kill -KILL -- "$target" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  if [[ -n "${bag_pid:-}" ]]; then
    stop_process "$bag_pid" "rosbag recorder"
  fi
  if [[ -n "${stack_pid:-}" ]]; then
    stop_process "$stack_pid" "IGVC Fortress stack" "group"
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch igvc_competition_sim igvc_competition.launch.py \
  course_config:="$COURSE_CONFIG" \
  ground_truth_pca:="$GROUND_TRUTH_PCA" \
  line_detection_mode:="$LINE_DETECTION_MODE" \
  nav2_params:="$NAV2_PARAMS" \
  launch_gazebo:="$LAUNCH_GAZEBO" \
  gazebo_server_only:="$GAZEBO_SERVER_ONLY" \
  launch_bridge:="$LAUNCH_BRIDGE" &
stack_pid=$!

sleep "$STARTUP_WAIT_SEC"

ros2 bag record --include-hidden-topics \
  -o "$RUN_DIR/bag" \
  /clock \
  /tf \
  /tf_static \
  /model/shogi/odometry \
  /odom \
  /local_ekf/odom \
  /joint_states \
  /cmd_vel \
  /cmd_vel_nav \
  /autonomous_mode \
  /gps_fix \
  /gps_waypoint/health \
  /gps_waypoint/debug \
  /goal_pose \
  /goal_update \
  /nav_goal \
  /cloud_all_fields_fullframe \
  /scan_fullframe \
  /scan_pca_filtered \
  /scan_pca_filtered_clear \
  /scan_pca_filtered_points \
  /terrain/grade_map \
  /pca/surface_normal \
  /igvc_sim/zed/image \
  /igvc_sim/zed/depth_image \
  /igvc_sim/zed/camera_info \
  /zed/zed_node/rgb/color/rect/image \
  /zed/zed_node/depth/depth_registered \
  /zed/zed_node/rgb/color/rect/camera_info \
  /zed/zed_node/depth/depth_info \
  /line_points \
  /line_costmap \
  /line_detection/line_pixels \
  /line_detection/diagnostics \
  /line_detection/debug/raw \
  /line_detection/debug/mask \
  /line_detection/debug/overlay \
  /lines_pointcloud \
  /lidar_line_points \
  /lidar_line_costmap \
  /lidar_line_detection/diagnostics \
  /lidar_line_detection/debug/points \
  /local_costmap/costmap \
  /local_costmap/costmap_raw \
  /global_costmap/costmap \
  /global_costmap/costmap_raw \
  /plan \
  /local_plan \
  /trajectories \
  /transformed_global_plan \
  /evaluation \
  /igvc_sim/score \
  /igvc_sim/fail \
  /navigate_to_waypoint/_action/status \
  /navigate_to_pose/_action/status \
  /follow_path/_action/status \
  /compute_path_to_pose/_action/status &
bag_pid=$!

sleep "$PRE_MISSION_WAIT_SEC"

mission_status=0
set +e
timeout --foreground "${MISSION_TIMEOUT_SEC}s" \
  ros2 run igvc_competition_sim igvc_mission_runner \
    --course-config "$COURSE_CONFIG" \
    --timeout-sec "$MISSION_TIMEOUT_SEC" \
  | tee "$RUN_DIR/mission.log"
mission_status="${PIPESTATUS[0]}"
set -e

set +e
ros2 topic echo --once /igvc_sim/score > "$RUN_DIR/final_score.txt" 2>&1
set -e

sleep 2
cleanup
set -e
trap - EXIT INT TERM

if [[ "$mission_status" -ne 0 ]]; then
  exit "$mission_status"
fi

if grep -q '"failed": true' "$RUN_DIR/final_score.txt"; then
  echo "IGVC monitor reported a failure. See $RUN_DIR/final_score.txt" >&2
  exit 1
fi

echo "IGVC Fortress run complete: $RUN_DIR"
