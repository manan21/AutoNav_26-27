#!/bin/bash
ros2 run gps_handler gps_publisher &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 5
echo "[GUI_READY] GPS"

wait "$launchpid"
