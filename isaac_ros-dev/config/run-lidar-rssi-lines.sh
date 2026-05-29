#!/bin/bash
# Standalone bring-up for plain-white-tape LiDAR RSSI line detection.
# This uses the alternate RSSI/intensity config and leaves the default
# retroreflector profile untouched.

set -euo pipefail

PARAMS="$(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/lidar_line_detector_rssi.yaml"

ros2 run autonav_detection lidar_line_detector --ros-args --params-file "$PARAMS" "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] LIDAR RSSI LINE DETECT"

wait "$launchpid"
