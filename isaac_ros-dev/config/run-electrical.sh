#!/bin/bash
# Run the electrical publisher and emit [GUI_READY] Power PCB once
# /electrical/voltage starts publishing.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

ros2 launch autonav_electrical_publisher electrical_publisher.launch.py &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "Power PCB" /electrical/voltage \
    --type std_msgs/msg/Float32 --qos sensor --timeout 30

wait "$launchpid"
