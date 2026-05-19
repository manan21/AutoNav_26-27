#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_PATH="${SCRIPT_DIR}/../src/slam/config/nav2_paramsv2.yaml"
BT_PATH="${SCRIPT_DIR}/../install/slam/share/slam/behavior_trees/bt_nav.xml"

ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$NAV_PATH \
  use_sim_time:=false \
  default_bt_xml_filename:=$BT_PATH &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] NAV2"

wait "$launchpid"
