#!/usr/bin/env bash
# run_mission.sh — sequence a chain of waypoints from stored_waypoints.txt.
#
# Usage: ./run_mission.sh [mission_file]
#   mission_file  defaults to stored_waypoints.txt next to this script.
#
# Each leg blocks until its action returns terminal status. The first
# leg that aborts stops the mission and exits non-zero.

set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MISSION=${1:-"$SCRIPT_DIR/stored_waypoints.txt"}

if [[ ! -f $MISSION ]]; then
  echo "mission file not found: $MISSION" >&2
  exit 1
fi

SEND_LOCAL="$SCRIPT_DIR/send_goal.sh"
SEND_GPS="$SCRIPT_DIR/send_GPS_waypoint.sh"
for sender in "$SEND_LOCAL" "$SEND_GPS"; do
  if [[ ! -x $sender ]]; then
    echo "sender not executable: $sender" >&2
    exit 1
  fi
done

# Count active legs for progress reporting.
TOTAL=$(grep -cE '^[[:space:]]*[A-Z]{2}[[:space:]]' "$MISSION" || true)
if [[ $TOTAL -eq 0 ]]; then
  echo "no active legs in $MISSION (only comments / blanks). Nothing to do." >&2
  exit 0
fi

# ── Pre-mission green-light gate ─────────────────────────────────────
# GPS waypoint navigation only works once every upstream signal is
# live (AUTO mode, GPS fix, global EKF, gps_handler, TF, action
# server). Block here until mission_precheck.py reports ALL GREEN.
# Override knobs:
#   MISSION_PRECHECK=0           skip the gate entirely (dev only)
#   MISSION_PRECHECK_TIMEOUT=N   seconds to wait for green-light
#   MISSION_PRECHECK_ARGS="..."  extra args passed to the checker
PRECHECK="$SCRIPT_DIR/mission_precheck.py"
if [[ ${MISSION_PRECHECK:-1} != "0" ]]; then
  if [[ ! -f $PRECHECK ]]; then
    echo "pre-check script missing: $PRECHECK" >&2
    exit 1
  fi
  echo "running mission pre-check before dispatch..."
  if ! python3 "$PRECHECK" \
        --timeout "${MISSION_PRECHECK_TIMEOUT:-60}" \
        ${MISSION_PRECHECK_ARGS:-}; then
    echo "mission pre-check FAILED — not starting mission." >&2
    exit 1
  fi
else
  echo "MISSION_PRECHECK=0 — skipping pre-mission green-light gate." >&2
fi

i=0
while IFS= read -r raw || [[ -n $raw ]]; do
  line=${raw%%#*}                           # strip inline comments
  line=${line//$'\r'/}                      # strip CRs
  [[ -z ${line//[[:space:]]/} ]] && continue

  # Normalize separators: replace commas with spaces, then split.
  normalized=${line//,/ }
  read -r kind a b extra <<< "$normalized"

  if [[ -n ${extra:-} ]]; then
    echo "leg has extra fields: $raw" >&2
    exit 1
  fi
  if [[ -z ${a:-} || -z ${b:-} ]]; then
    echo "leg missing values: $raw" >&2
    exit 1
  fi

  i=$((i + 1))
  echo "[$i/$TOTAL] $kind $a $b"

  case $kind in
    ER) "$SEND_LOCAL" -r "$a" "$b" ;;
    EA) "$SEND_LOCAL" "$a" "$b" ;;
    GA) "$SEND_GPS" "$a" "$b" ;;
    *)
      echo "unknown leg type '$kind' in: $raw" >&2
      exit 1
      ;;
  esac
  status=$?

  if [[ $status -ne 0 ]]; then
    echo "leg $i ($kind $a $b) failed with status $status — stopping mission." >&2
    exit "$status"
  fi
done < "$MISSION"

echo "mission complete: $i/$TOTAL legs succeeded."
