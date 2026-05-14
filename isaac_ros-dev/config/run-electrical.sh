#!/bin/bash
ros2 launch autonav_electrical_publisher electrical_publisher.launch.py &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] Power PCB"

wait "$launchpid"
