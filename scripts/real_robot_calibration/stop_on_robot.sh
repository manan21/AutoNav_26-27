#!/usr/bin/env bash
# Jetson-side emergency/cleanup helper for the active calibration run.

set -euo pipefail

STATE_DIR=${AUTONAV_CALIB_STATE_DIR:-$HOME/.autonav_real_robot_calibration}
ACTIVE_FILE="$STATE_DIR/active_run.env"

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    local status=$?
    set -u
    return "$status"
  fi
}

publish_zero() {
  if command -v ros2 >/dev/null 2>&1; then
    timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
      >/dev/null 2>&1 || true
  fi
}

source_setup /opt/ros/humble/setup.bash
source_setup "$HOME/AutoNav_25-26/isaac_ros-dev/install/setup.bash"
source_setup "$HOME/code/git/AutoNav_25-26/isaac_ros-dev/install/setup.bash"
source_setup "/workspaces/isaac_ros-dev/install/setup.bash"

publish_zero

if [ ! -f "$ACTIVE_FILE" ]; then
  echo "No active calibration run file found at $ACTIVE_FILE."
  echo "Published zero /cmd_vel anyway."
  exit 0
fi

# shellcheck disable=SC1090
source "$ACTIVE_FILE"

echo "Requesting stop for active calibration run:"
echo "  runner pid: ${RUNNER_PID:-unknown}"
echo "  bag path: ${BAG_PATH:-unknown}"

if [ -n "${RUNNER_PID:-}" ] && kill -0 "$RUNNER_PID" >/dev/null 2>&1; then
  kill -INT "$RUNNER_PID" >/dev/null 2>&1 || true
  sleep 2
fi

publish_zero

if [ -n "${BAG_PID:-}" ] && kill -0 "$BAG_PID" >/dev/null 2>&1; then
  kill -INT "$BAG_PID" >/dev/null 2>&1 || true
fi

rm -f "$ACTIVE_FILE"
echo "Stop requested. If a tmux session is still open, attach to confirm final ros2 bag info."
