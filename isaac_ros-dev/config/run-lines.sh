#!/bin/bash
# Standalone bring-up for just the line detector. Loads parameters from
# the package-share YAML; override individual values via --ros-args at
# the command line. For both detectors at once, use ./config/run-detect.sh.
#
# Emits [GUI_READY] LINE DETECT once /lines_pointcloud starts publishing.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

PARAMS="$(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/line_detector.yaml"

ros2 run autonav_detection line_detector --ros-args --params-file "$PARAMS" "$@" &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "LINE DETECT" /lines_pointcloud \
    --type sensor_msgs/msg/PointCloud2 --qos sensor --timeout 45

wait "$launchpid"
