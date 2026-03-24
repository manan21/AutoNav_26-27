#!/bin/bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:='zed2i' \
  publish_tf:=false \
  publish_map_tf:=false
