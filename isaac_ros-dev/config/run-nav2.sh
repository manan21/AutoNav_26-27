#!/bin/bash

NAV_PATH="$(dirname ${BASH_SOURCE[0]})/../src/slam/config/nav2_paramsv2.yaml"

ros2 launch slam nav.launch.py nav2_params:=$NAV_PATH use_sim_time:=false
