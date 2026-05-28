#!/bin/bash
# Run the standard post-test analysis suite for a lidar-line avoidance bag.

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 /path/to/bag [extra analyzer args are not supported]" >&2
  exit 1
fi

BAG_PATH=$1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

run_step() {
  echo
  echo "== $* =="
  "$@"
}

run_optional_step() {
  echo
  echo "== optional: $* =="
  set +e
  "$@"
  local status=$?
  set -e
  if [ "$status" -ne 0 ]; then
    echo "Optional analyzer exited with status $status; continuing."
  fi
}

run_step python3 "$SCRIPT_DIR/analyze_lidar_line_bag.py" "$BAG_PATH"
run_step python3 "$SCRIPT_DIR/analyze_lidar_line_timeline.py" "$BAG_PATH"
run_step python3 "$SCRIPT_DIR/analyze_nav2_action_result.py" "$BAG_PATH" --require-succeeded --require-cmd-vel-nav --warn-follow-path-aborts
run_step python3 "$SCRIPT_DIR/analyze_lidar_line_plan_gap.py" "$BAG_PATH" --perp-x 1.34 --tape-right-y -0.13 --half-width 0.46 --fail-without-gap-plan
run_step python3 "$SCRIPT_DIR/analyze_global_plan_costmap_collision.py" "$BAG_PATH" --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 1.524 --tape-right-y -0.13 --half-length 0.595 --half-width 0.46
run_step python3 "$SCRIPT_DIR/analyze_lidar_line_course_clearance.py" "$BAG_PATH" --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 1.524 --padding 0.05 --fail-on-overlap
run_step python3 "$SCRIPT_DIR/analyze_executed_footprint_costmap_collision.py" "$BAG_PATH" --half-length 0.595 --half-width 0.46 --require-costmap --fail-on-overlap
run_step python3 "$SCRIPT_DIR/analyze_costmap_footprint.py" "$BAG_PATH" --half-length 0.595 --half-width 0.46 --hard-threshold 100
run_optional_step python3 "$SCRIPT_DIR/analyze_dwb_evaluation.py" "$BAG_PATH" --window 0.1
