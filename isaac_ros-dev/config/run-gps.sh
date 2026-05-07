#!/bin/bash
# Run the GPS publisher and emit [GUI_READY] GPS once /gps_fix is publishing.
# Outdoor first-fix can take minutes, so the wait timeout matches the HUD's
# device readiness window (300s).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

ros2 run gps_handler gps_publisher &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "GPS" /gps_fix \
    --type sensor_msgs/msg/NavSatFix --qos sensor --timeout 290

wait "$launchpid"
