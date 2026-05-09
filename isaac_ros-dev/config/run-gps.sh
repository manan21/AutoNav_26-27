#!/bin/bash
ros2 run gps_handler gps_publisher &
pid_pub=$!

ros2 run gps_waypoint_handler gps_handler_node &
pid_handler=$!

trap 'kill -INT "$pid_pub" "$pid_handler" 2>/dev/null' INT TERM

sleep 5
echo "[GUI_READY] GPS"

wait
