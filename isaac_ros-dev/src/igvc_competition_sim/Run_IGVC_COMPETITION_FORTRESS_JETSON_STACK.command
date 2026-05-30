#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTONAV_REPO="${AUTONAV_REPO:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
ROS_WS="${ROS_WS:-$AUTONAV_REPO/isaac_ros-dev}"
COURSE_CONFIG="${COURSE_CONFIG:-$ROS_WS/install/igvc_competition_sim/share/igvc_competition_sim/config/igvc_competition_compact.yaml}"
DYNAMICS_CALIBRATION="${DYNAMICS_CALIBRATION:-$ROS_WS/install/igvc_competition_sim/share/igvc_competition_sim/config/dynamics_calibration.yaml}"
LINE_DETECTION_MODE="${LINE_DETECTION_MODE:-camera}"
GROUND_TRUTH_PCA="${GROUND_TRUTH_PCA:-false}"
USE_CALIBRATED_DYNAMICS="${USE_CALIBRATED_DYNAMICS:-true}"
LAUNCH_MONITOR="${LAUNCH_MONITOR:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

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

export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION

exec ros2 launch igvc_competition_sim igvc_competition.launch.py \
  course_config:="$COURSE_CONFIG" \
  ground_truth_pca:="$GROUND_TRUTH_PCA" \
  line_detection_mode:="$LINE_DETECTION_MODE" \
  nav2_params:="$NAV2_PARAMS" \
  use_calibrated_dynamics:="$USE_CALIBRATED_DYNAMICS" \
  dynamics_calibration:="$DYNAMICS_CALIBRATION" \
  launch_gazebo:=false \
  launch_bridge:=false \
  launch_camera_bridge:=true \
  launch_harness:=true \
  launch_odom_bridge:=true \
  publish_harness_odom_tf:=false \
  launch_dynamics:=true \
  launch_robot_state_publisher:=true \
  launch_detection:=true \
  launch_monitor:="$LAUNCH_MONITOR" \
  launch_pca_scan_converters:=true \
  launch_gps_handler:=true \
  launch_nav:=true
