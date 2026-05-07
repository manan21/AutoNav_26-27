#!/bin/bash
# Bring up the pre-SLAM stack (joy, core, control, wheel odometry) and emit
# [GUI_READY] Pre-SLAM once /odom is publishing — the last node in the
# launch group to come online (control after 1s, odom after 5s).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

ros2 launch bringup pre_slam.launch.py &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "Pre-SLAM" /odom \
    --type nav_msgs/msg/Odometry --qos sensor --timeout 60

wait "$launchpid"
