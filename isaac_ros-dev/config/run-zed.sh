#!/bin/bash
# Bring up the ZED 2i camera and emit [GUI_READY] Camera once an RGB image
# has actually been published.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:='zed2i' \
  publish_tf:=false \
  publish_map_tf:=false \
  ros_params_override_path:=/autonav/isaac_ros-dev/install/bringup/share/bringup/config/zed_override.yaml &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "Camera" /zed/zed_node/rgb/image_rect_color \
    --type sensor_msgs/msg/Image --qos sensor --timeout 45

wait "$launchpid"
