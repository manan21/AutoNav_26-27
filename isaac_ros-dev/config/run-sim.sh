#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${ROS_WS:-}" ]]; then
  SPLIT_WS_CANDIDATE="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
  LEGACY_WS_CANDIDATE="$(cd "$SCRIPT_DIR/.." && pwd)"
  if [[ -f "$SPLIT_WS_CANDIDATE/install/setup.bash" ]]; then
    ROS_WS="$SPLIT_WS_CANDIDATE"
  else
    ROS_WS="$LEGACY_WS_CANDIDATE"
  fi
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "Workspace install setup not found:" >&2
  echo "  $ROS_WS/install/setup.bash" >&2
  echo "Build the split workspace first: cd ~/autonav_ws && colcon build --symlink-install" >&2
  echo "The Gazebo sim now lives in https://github.com/blobspire/autonav_sim." >&2
  exit 1
fi

set +u
source "$ROS_WS/install/setup.bash"
set -u

if ! ros2 pkg prefix igvc_competition_sim >/dev/null 2>&1; then
  echo "ROS package igvc_competition_sim was not found in this workspace." >&2
  echo "Clone/build https://github.com/blobspire/autonav_sim beside AutoNav_25-26." >&2
  exit 1
fi

exec ros2 launch igvc_competition_sim igvc_competition.launch.py "$@"
