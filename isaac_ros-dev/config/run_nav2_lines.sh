#!/bin/bash

NAV_PATH="$(dirname ${BASH_SOURCE[0]})/../src/slam/config/nav2_lines_no_slam_params.yaml"

ros2 launch slam nav_lines_no_slam.launch.py nav2_params:=$NAV_PATH use_sim_time:=false auto_nav_goal:=true
