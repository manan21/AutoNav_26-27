#!/bin/bash
ros2 launch bringup pre_slam.launch.py &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] Pre-SLAM"

wait "$launchpid"
