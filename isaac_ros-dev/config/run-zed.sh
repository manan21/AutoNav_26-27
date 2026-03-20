#!/bin/bash
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:='zed2i' \
  ros_params_override_path:="$SCRIPT_DIR/zed_static_override.yaml"
