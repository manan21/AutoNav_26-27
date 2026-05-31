#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "AutoNav workspace install setup not found:" >&2
  echo "  $ROS_WS/install/setup.bash" >&2
  echo "Build the workspace first: cd $ROS_WS && colcon build" >&2
  exit 1
fi

set +u
source "$ROS_WS/install/setup.bash"
set -u

exec ros2 launch igvc_competition_sim igvc_competition.launch.py "$@"
