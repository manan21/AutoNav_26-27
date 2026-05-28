#!/bin/bash
# Brings up perception detectors via the autonav_detection package's
# launch file. Parameters are loaded from
# the package-share config/ directory; override at the command line via:
#   ./config/run-detect.sh enable_grade:=false
#   ./config/run-detect.sh enable_lidar_line:=false
#   ./config/run-detect.sh grade_detector_params:=/path/to/custom.yaml
ros2 launch autonav_detection detection.launch.py "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] DETECT"

wait "$launchpid"
