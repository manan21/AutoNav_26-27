#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_PATH="${SCRIPT_DIR}/../src/slam/config/nav2_paramsv2.yaml"
BT_PATH="${SCRIPT_DIR}/../src/slam/behavior_trees/bt_nav.xml"
# Bond-timeout-patched wrapper around nav2_bringup navigation_launch.py
# (raises lifecycle_manager bond_timeout 4s -> 20s so CPU-starved servers
# are not torn down mid-mission; see the launch file's docstring). Launched
# by absolute path so no colcon rebuild is required to pick it up.
LAUNCH_PATH="${SCRIPT_DIR}/../src/slam/launch/nav2_navigation.launch.py"

ros2 launch "$LAUNCH_PATH" \
  params_file:=$NAV_PATH \
  use_sim_time:=false \
  default_bt_xml_filename:=$BT_PATH &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] NAV2"

wait "$launchpid"
