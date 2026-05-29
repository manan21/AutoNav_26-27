#!/bin/bash
# Run the standard post-test analysis suite for a lidar-line avoidance bag.

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 /path/to/bag [--scenario-config path] [--strict-scenario-geometry]" >&2
  exit 1
fi

BAG_PATH=$1
shift
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENARIO_CONFIG=""
STRICT_SCENARIO_GEOMETRY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --scenario-config)
      SCENARIO_CONFIG="${2:-}"
      shift 2
      ;;
    --strict-scenario-geometry)
      STRICT_SCENARIO_GEOMETRY=1
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

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
run_step python3 "$SCRIPT_DIR/analyze_nav2_action_result.py" "$BAG_PATH" --require-succeeded --require-cmd-vel-nav --warn-follow-path-aborts
run_step python3 "$SCRIPT_DIR/analyze_bt_control_churn.py" "$BAG_PATH"

scenario_id=""
if [ -n "$SCENARIO_CONFIG" ]; then
  scenario_id="$(python3 - "$SCENARIO_CONFIG" <<'PY'
from pathlib import Path
import sys

legacy = False
for line in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines():
    stripped = line.split("#", 1)[0].strip()
    if not stripped or ":" not in stripped:
        continue
    key, value = stripped.split(":", 1)
    key = key.strip()
    if key == "scenario_id":
        print(value.strip().strip("\"'"))
        break
    if key == "perpendicular_tape_lidar_x_m":
        legacy = True
else:
    if legacy:
        print("canonical_5ft_gap")
PY
)"
  scenario_args=(--scenario-config "$SCENARIO_CONFIG" --fail-on-overlap)
  if [ "$STRICT_SCENARIO_GEOMETRY" -eq 1 ]; then
    scenario_args+=(--strict-stations --fail-on-padded-overlap)
  fi
  run_step python3 "$SCRIPT_DIR/analyze_lidar_line_scenario.py" "$BAG_PATH" "${scenario_args[@]}"
  if [ "$scenario_id" = "driveway_1in_wall_gap_repro" ]; then
    run_step python3 "$SCRIPT_DIR/analyze_driveway_wall_gap_repro.py" "$BAG_PATH" \
      --scenario-config "$SCENARIO_CONFIG" \
      --fail-on-gap-plan \
      --fail-on-masked-wall
  fi
fi

if [ -z "$SCENARIO_CONFIG" ] || [ "$scenario_id" = "canonical_5ft_gap" ]; then
  run_step python3 "$SCRIPT_DIR/analyze_lidar_line_timeline.py" "$BAG_PATH"
  run_step python3 "$SCRIPT_DIR/analyze_lidar_line_plan_gap.py" "$BAG_PATH" --perp-x 1.34 --tape-right-y -0.13 --half-width 0.46 --fail-without-gap-plan
  run_step python3 "$SCRIPT_DIR/analyze_global_plan_costmap_collision.py" "$BAG_PATH" --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 1.524 --tape-right-y -0.13 --half-length 0.595 --half-width 0.46
  run_step python3 "$SCRIPT_DIR/analyze_lidar_line_course_clearance.py" "$BAG_PATH" --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 1.524 --padding 0.05 --fail-on-overlap
fi

run_step python3 "$SCRIPT_DIR/analyze_executed_footprint_costmap_collision.py" "$BAG_PATH" --half-length 0.595 --half-width 0.46 --require-costmap --fail-on-overlap
run_step python3 "$SCRIPT_DIR/analyze_costmap_footprint.py" "$BAG_PATH" --half-length 0.595 --half-width 0.46 --hard-threshold 100
run_optional_step python3 "$SCRIPT_DIR/analyze_dwb_evaluation.py" "$BAG_PATH" --window 0.1
