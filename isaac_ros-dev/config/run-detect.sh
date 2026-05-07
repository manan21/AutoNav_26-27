#!/bin/bash
# Brings up both perception detectors (line + PCA grade) via the
# autonav_detection package's launch file. Parameters are loaded from
# the package-share config/ directory; override at the command line via:
#   ./config/run-detect.sh enable_grade:=false
#   ./config/run-detect.sh grade_detector_params:=/path/to/custom.yaml
ros2 launch autonav_detection detection.launch.py "$@"
