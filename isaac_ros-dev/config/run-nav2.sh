#!/bin/bash
# Bring up Nav2 with the project's nav2 params and BT, emit [GUI_READY] NAV2
# once the global costmap has actually published its first occupancy grid.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

NAV_PATH="${SCRIPT_DIR}/../src/slam/config/nav2_paramsv2.yaml"
BT_PATH="${SCRIPT_DIR}/../install/slam/share/slam/behavior_trees/bt_nav.xml"

ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$NAV_PATH \
  use_sim_time:=false \
  default_bt_xml_filename:=$BT_PATH &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "NAV2" /global_costmap/costmap \
    --type nav_msgs/msg/OccupancyGrid --qos transient_local --timeout 90

wait "$launchpid"
