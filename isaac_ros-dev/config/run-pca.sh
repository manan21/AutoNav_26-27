#!/bin/bash
# Standalone bring-up for just the PCA grade detector (SICK PointCloud2
# → PCA-filtered obstacle cloud). Loads parameters from the package-share
# YAML. For both detectors at once, use ./config/run-detect.sh.

PARAMS="$(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/grade_detector.yaml"

ros2 run autonav_detection grade_detector --ros-args --params-file "$PARAMS" "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] PCA DETECT"

wait "$launchpid"
