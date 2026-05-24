#!/bin/bash
# Standalone bring-up for the LiDAR RSSI line detector. This is opt-in
# so camera line detection remains unchanged while LiDAR lines are tuned.

PARAMS="$(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/lidar_line_detector.yaml"

ros2 run autonav_detection lidar_line_detector --ros-args --params-file "$PARAMS" "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] LIDAR LINE DETECT"

wait "$launchpid"
