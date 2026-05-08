#!/usr/bin/env bash
# isaac_ros-dev/config/send_GPS_waypoint.sh
# Usage: ./send_GPS_waypoint.sh <lat> <lon> [radius_m]
#
# Sends a NavigateToWaypoint goal and streams feedback. On Ctrl+C (SIGINT)
# or SIGTERM, parses the goal UUID from send_goal's stdout and issues a
# `ros2 action cancel` so the cancellation reaches the action server,
# not just the local CLI process. This is the workaround for the fact
# that `ros2 action send_goal` in ROS 2 Humble has NO --cancel-on-disconnect
# flag (that flag does not exist in the Humble CLI).

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <lat> <lon> [radius_m]" >&2
  exit 1
fi

LAT=$1
LON=$2
RADIUS=${3:-1.0}

# Validate that lat/lon are decimal numbers (signed, optional fraction).
if ! [[ $LAT =~ ^-?[0-9]+(\.[0-9]+)?$ && $LON =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
  echo "lat and lon must be decimal numbers" >&2
  exit 1
fi
if ! [[ $RADIUS =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
  echo "radius_m must be a decimal number" >&2
  exit 1
fi

GOAL_YAML="{goal_type: 0, target: {header: {frame_id: 'wgs84'}, pose: {position: {x: $LON, y: $LAT, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, success_radius_m: $RADIUS}"

# Tempfile to capture send_goal's stdout so the trap can parse the UUID.
OUT_LOG=$(mktemp -t send_gps_waypoint.XXXXXX)

cleanup_tmp() { rm -f "$OUT_LOG"; }
trap cleanup_tmp EXIT

# Run send_goal in the background, teeing stdout so the operator still
# sees feedback in real time AND we have a parseable copy in OUT_LOG.
ros2 action send_goal /navigate_to_waypoint \
  autonav_interfaces/action/NavigateToWaypoint \
  "$GOAL_YAML" \
  --feedback 2>&1 | tee "$OUT_LOG" &
SEND_PID=$!

on_interrupt() {
  echo "" >&2
  echo "interrupt received -- cancelling /navigate_to_waypoint" >&2

  # The goal UUID is printed by `ros2 action send_goal` once the goal is
  # accepted. The exact line in Humble looks like:
  #   "Goal accepted with ID: a1b2c3d4e5f6..."
  # We grep for that and extract the trailing token. If the goal hasn't
  # been accepted yet, UUID will be empty and we skip the cancel.
  UUID=$(grep -E 'Goal accepted with ID' "$OUT_LOG" 2>/dev/null \
           | tail -n1 \
           | awk -F': ' '{print $2}' \
           | tr -d '[:space:]' || true)

  if [[ -n "${UUID:-}" ]]; then
    # NOTE: `ros2 action cancel <action_name> <uuid>` is the documented
    # Humble form. If your Humble build's CLI rejects this signature,
    # try `ros2 action cancel <action_name>` (some distros allow omitting
    # the UUID to cancel all active goals). Adjust as needed.
    ros2 action cancel /navigate_to_waypoint "$UUID" 2>/dev/null || \
      echo "ros2 action cancel failed (UUID=$UUID); falling through to SIGTERM" >&2
  else
    echo "no goal UUID parsed from send_goal output; skipping action cancel" >&2
  fi

  # Always tear down the local CLI process so the script returns promptly.
  kill -TERM "$SEND_PID" 2>/dev/null || true
  wait "$SEND_PID" 2>/dev/null || true
  exit 130
}
trap on_interrupt INT TERM

# Wait for send_goal to finish on its own (goal succeeded / aborted /
# rejected). `set -e` is fine here because we explicitly capture $?.
set +e
wait "$SEND_PID"
EXIT=$?
set -e
trap - INT TERM
exit $EXIT
