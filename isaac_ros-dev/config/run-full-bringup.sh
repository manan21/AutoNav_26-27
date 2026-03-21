#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi

PIDS=()

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if ((${#PIDS[@]})); then
    echo
    echo "Stopping bringup processes..."
    kill "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi

  exit "$exit_code"
}

launch_bg() {
  local name=$1
  shift

  echo "Starting ${name}..."
  "$@" &
  local pid=$!
  PIDS+=("$pid")
  echo "${name} pid=${pid}"
}

trap cleanup EXIT INT TERM

launch_bg "pre_slam" ros2 launch bringup pre_slam.launch.py
sleep 2

launch_bg "zed" "$SCRIPT_DIR/run-zed.sh"
sleep 3

launch_bg "lidar" "$SCRIPT_DIR/run-lidar.sh"
sleep 3

launch_bg "slam" ros2 launch slam slam.launch.py
sleep 5

launch_bg "lines" "$SCRIPT_DIR/run-lines.sh"

echo

echo "Full bringup running. Press Ctrl+C to stop all processes."
wait -n
