#!/bin/bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:='zed2i' \
  publish_tf:=false \
  publish_map_tf:=false \
  ros_params_override_path:=/autonav/isaac_ros-dev/install/bringup/share/bringup/config/zed_override.yaml &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] Camera"

wait "$launchpid"
