#!/bin/bash
PARAMS="$(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/line_detector.yaml"

ros2 run autonav_detection line_detector --ros-args --params-file "$PARAMS" "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 5
echo "[GUI_READY] LINE DETECT"

wait "$launchpid"
