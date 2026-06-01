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

# GI (GPS Insert) records the robot's current GPS position to this file
# by averaging N /gps_fix samples; a later GR (GPS Return) leg reads it
# back and dispatches send_GPS_waypoint.sh. The recorder intentionally
# has no timeout — see record_gps_here.py for the rationale.
GI_RECORDER="$SCRIPT_DIR/record_gps_here.py"
GI_WAYPOINT_FILE="${GI_WAYPOINT_FILE:-$SCRIPT_DIR/gi_waypoint.txt}"
GI_SAMPLES="${GI_SAMPLES:-10}"

# Count active legs for progress reporting. Match both two-arg kinds
# (ER/EA/GA, followed by whitespace) and argless kinds (GI/GR, alone
# on the line); without the `($|[[:space:]])` branch a trailing GI/GR
# would be invisible to the progress counter.
TOTAL=$(grep -cE '^[[:space:]]*[A-Z]{2}([[:space:]]|$)' "$MISSION" || true)
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
ok=0
aborted=0
while IFS= read -r raw || [[ -n $raw ]]; do
  line=${raw%%#*}                           # strip inline comments
  line=${line//$'\r'/}                      # strip CRs
  [[ -z ${line//[[:space:]]/} ]] && continue

  # Normalize separators: replace commas with spaces, then split.
  normalized=${line//,/ }
  read -r kind a b extra <<< "$normalized"

  i=$((i + 1))
  status=0

  case $kind in
    GI)
      # Argless: record-and-store the robot's current GPS position so a
      # later GR leg can return to it. Per-leg failures (bad args, no
      # GPS) flow through the "log + continue" path below; the mission
      # never skips ahead because of a problem on this leg.
      if [[ -n ${a:-} ]]; then
        echo "[$i/$TOTAL] GI takes no arguments: $raw" >&2
        status=1
      else
        echo "[$i/$TOTAL] GI (recording current GPS, ${GI_SAMPLES} samples)"
        python3 "$GI_RECORDER" \
          --samples "$GI_SAMPLES" \
          "$GI_WAYPOINT_FILE"
        status=$?
      fi
      ;;
    GR)
      # Argless: replay the GI-recorded waypoint as a GA-style goal.
      # Same policy as GI — any precondition gap is logged and we move
      # on; the mission file's remaining legs still get dispatched.
      if [[ -n ${a:-} ]]; then
        echo "[$i/$TOTAL] GR takes no arguments: $raw" >&2
        status=1
      elif [[ ! -f $GI_WAYPOINT_FILE ]]; then
        echo "[$i/$TOTAL] GR before GI — no recorded waypoint at $GI_WAYPOINT_FILE" >&2
        status=1
      else
        read -r r_lat r_lon _rest < "$GI_WAYPOINT_FILE" || true
        if [[ -z ${r_lat:-} || -z ${r_lon:-} ]]; then
          echo "[$i/$TOTAL] GR: malformed recorded waypoint in $GI_WAYPOINT_FILE" >&2
          status=1
        else
          echo "[$i/$TOTAL] GR (returning to $r_lat $r_lon)"
          "$SEND_GPS" "$r_lat" "$r_lon"
          status=$?
        fi
      fi
      ;;
    ER|EA|GA)
      if [[ -n ${extra:-} ]]; then
        echo "[$i/$TOTAL] leg has extra fields: $raw" >&2
        status=1
      elif [[ -z ${a:-} || -z ${b:-} ]]; then
        echo "[$i/$TOTAL] leg missing values: $raw" >&2
        status=1
      else
        echo "[$i/$TOTAL] $kind $a $b"
        case $kind in
          ER) "$SEND_LOCAL" -r "$a" "$b" ;;
          EA) "$SEND_LOCAL" "$a" "$b" ;;
          GA) "$SEND_GPS" "$a" "$b" ;;
        esac
        status=$?
      fi
      ;;
    *)
      echo "[$i/$TOTAL] unknown leg type '$kind' in: $raw" >&2
      status=1
      ;;
  esac

  # Mission policy: a Nav2 abort on one leg is not a mission-level
  # failure — the operator wants the chain to keep dispatching the next
  # waypoint regardless. The only exit we treat as "stop the whole
  # mission" is 130 (the SIGINT/SIGTERM trap path inside the dispatchers,
  # i.e. operator-initiated cancel). Everything else gets logged loudly
  # and we move on.
  if [[ $status -eq 0 ]]; then
    ok=$((ok + 1))
  elif [[ $status -eq 130 ]]; then
    echo "leg $i ($raw) interrupted by operator (exit 130) — stopping mission." >&2
    exit 130
  else
    aborted=$((aborted + 1))
    echo "leg $i ($raw) returned status $status — continuing to next leg." >&2
  fi
done < "$MISSION"

echo "mission complete: $ok/$TOTAL legs succeeded, $aborted aborted."
