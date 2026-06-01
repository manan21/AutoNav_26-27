#!/usr/bin/env bash
# run_mission.sh — sequence a chain of waypoints from stored_waypoints.txt.
#
# Usage: ./run_mission.sh [-n|-s] [mission_file]
#   -n            (default) execute GA waypoints in file order
#   -s            reverse the order of active GA legs at load time so
#                 the course runs in the opposite direction without
#                 having to edit the mission file. GI / GR / EA / ER
#                 legs and comments keep their original positions.
#   mission_file  defaults to stored_waypoints.txt next to this script.
#
# Per-leg policy: a leg returning non-zero is logged and the mission
# continues to the next leg. The only mid-mission stop is exit 130
# (SIGINT/SIGTERM caught by a dispatcher's own trap).

set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

DIRECTION=N
while getopts ":nsh" opt; do
  case $opt in
    n) DIRECTION=N ;;
    s) DIRECTION=S ;;
    h)
      sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    \?)
      echo "unknown option: -$OPTARG" >&2
      echo "usage: $0 [-n|-s] [mission_file]" >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))

MISSION=${1:-"$SCRIPT_DIR/stored_waypoints.txt"}

if [[ ! -f $MISSION ]]; then
  echo "mission file not found: $MISSION" >&2
  exit 1
fi

# Direction handling: -s reverses only the active GA legs (lines that
# match the leg-dispatch parser's expectation of `GA <a> <b>`). All
# other lines — GI, GR, EA, ER, comments, blanks — stay at their
# original positions. The reversed mission is materialized into a
# tempfile that becomes the new $MISSION; the dispatch loop below
# reads it normally without knowing whether reversal happened.
#
# Why a tempfile instead of an in-memory array fed to the loop:
# the existing `while read … < "$MISSION"` form is well tested
# (handles CRs, inline comments, trailing-newline-less files); a
# tempfile keeps that path untouched and makes the reversal visible
# for post-mortem ("what was actually dispatched?").
REVERSED_MISSION=""
cleanup_reversed() {
  if [[ -n "$REVERSED_MISSION" && -f "$REVERSED_MISSION" ]]; then
    rm -f "$REVERSED_MISSION"
  fi
}
trap cleanup_reversed EXIT

if [[ $DIRECTION == "S" ]]; then
  REVERSED_MISSION=$(mktemp -t mission_S.XXXXXX)
  # Portable read into array — matches the dispatch loop's own
  # `while IFS= read … || [[ -n $line ]]` pattern (which handles
  # files without a trailing newline) and keeps this script
  # buildable on bash 3.x as well as bash 5 on the Jetson.
  _LINES=()
  while IFS= read -r _line || [[ -n $_line ]]; do
    _LINES+=("$_line")
  done < "$MISSION"

  # Indices of active GA legs (commented-out or non-GA lines skipped).
  _ga_idx=()
  for _i in "${!_LINES[@]}"; do
    if [[ "${_LINES[_i]}" =~ ^[[:space:]]*GA[[:space:]] ]]; then
      _ga_idx+=("$_i")
    fi
  done

  # In-place reverse of the GA subsequence by swapping symmetric
  # pairs of indices. Handles odd counts — the middle GA stays put.
  _n=${#_ga_idx[@]}
  for ((_k = 0; _k < _n / 2; _k++)); do
    _lo=${_ga_idx[_k]}
    _hi=${_ga_idx[_n - 1 - _k]}
    _tmp=${_LINES[_lo]}
    _LINES[_lo]=${_LINES[_hi]}
    _LINES[_hi]=$_tmp
  done

  printf '%s\n' "${_LINES[@]}" > "$REVERSED_MISSION"
  MISSION=$REVERSED_MISSION
  echo "Direction: SOUTH — reversed $_n GA leg(s) (mission file at $REVERSED_MISSION)."
else
  echo "Direction: NORTH — GA legs in file order."
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
