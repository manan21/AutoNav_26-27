#!/bin/bash
# Standalone bring-up for just the line detector. Loads parameters from
# the package-share YAML; override individual values via --ros-args at
# the command line. For both detectors at once, use ./config/run-detect.sh.

PARAMS="$(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/line_detector.yaml"

ros2 run autonav_detection line_detector --ros-args --params-file "$PARAMS" "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 3
echo "[GUI_READY] LINE DETECT"

wait "$launchpid"
