#!/bin/bash
# Record one short raw LiDAR/RSSI bag for offline plain-white-tape analysis.
#
# Field procedure for limited course access:
#   1. Start this before entering the line area.
#   2. Hold still with tape visible for ~10-15 s.
#   3. Slowly drive or push past/along the tape.
#   4. Spend the last ~10-15 s looking at nearby non-line ground.
#
# The bag includes raw /cloud_all_fields_fullframe so RSSI thresholds can be
# swept offline without another course visit.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source_setup() {
  local setup_file=$1

  if [ ! -f "$setup_file" ]; then
    return 0
  fi

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  local source_status=$?
  set -u
  return "$source_status"
}

source_setup /opt/ros/humble/setup.bash
source_setup "$ISAAC_ROOT/install/setup.bash"

RUN_NAME="${1:-rssi_lines_$(date +%Y%m%d_%H%M%S)}"
LOG_ROOT="${RSSI_LINE_LOG_ROOT:-/autonav/logs/rssi_lines}"
RUN_DIR="$LOG_ROOT/$RUN_NAME"
BAG_PATH="$RUN_DIR/bag"
BAG_LOG="$RUN_DIR/rosbag_record.log"
DURATION="${RSSI_LINE_RECORD_SECONDS:-90}"

TOPICS=(
  /tf
  /tf_static
  /local_ekf/odom
  /odom
  /cloud_all_fields_fullframe
  /scan_fullframe
  /lidar_line_points
  /lidar_line_detection/debug/points
  /lidar_line_detection/diagnostics
)

mkdir -p "$RUN_DIR"

echo "RSSI line sample: $RUN_NAME"
echo "Bag path: $BAG_PATH"
echo "Duration: ${DURATION}s (set RSSI_LINE_RECORD_SECONDS=0 to record until Ctrl-C)"
echo
echo "Course choreography:"
echo "  0-15s: tape visible while stationary"
echo "  middle: slow pass across/along plain white tape"
echo "  final: nearby non-line ground for false-positive baseline"
echo

if [ "$DURATION" = "0" ]; then
  ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}" >"$BAG_LOG" 2>&1
else
  set +e
  timeout -s INT "$DURATION" ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}" >"$BAG_LOG" 2>&1
  status=$?
  set -e
  if [ "$status" -ne 0 ] && [ "$status" -ne 124 ] && [ "$status" -ne 130 ]; then
    echo "rosbag record failed with status $status; see $BAG_LOG" >&2
    exit "$status"
  fi
fi

echo
echo "Recorded bag: $BAG_PATH"
echo "Bag log: $BAG_LOG"
echo "Next:"
echo "  cd $ISAAC_ROOT/.."
echo "  python3 scripts/analyze_lidar_rssi_sweep.py $BAG_PATH"
