#!/usr/bin/env bash
# Laptop helper to stop the active robot-side calibration run.

set -euo pipefail

ROBOT=${AUTONAV_CALIB_ROBOT:-jetson}
REMOTE_SUITE_DIR=${AUTONAV_CALIB_REMOTE_SUITE_DIR:-}
SESSION=${AUTONAV_CALIB_TMUX_SESSION:-autonav_calib_bag}
ATTACH=0
if [ -z "$REMOTE_SUITE_DIR" ]; then
  REMOTE_SUITE_DIR='~/AutoNav_25-26/scripts/real_robot_calibration'
fi

usage() {
  cat >&2 <<'EOF'
Usage:
  stop_remote.sh [options]

Options:
  --robot HOST              SSH target, default: jetson
  --remote-suite-dir DIR    Robot staging dir, default: ~/AutoNav_25-26/scripts/real_robot_calibration
  --session NAME            tmux session name, default: autonav_calib_bag
  --attach                  Attach to the tmux session after requesting stop
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --robot)
      ROBOT=${2:?missing --robot value}
      shift 2
      ;;
    --remote-suite-dir)
      REMOTE_SUITE_DIR=${2:?missing --remote-suite-dir value}
      shift 2
      ;;
    --session)
      SESSION=${2:?missing --session value}
      shift 2
      ;;
    --attach)
      ATTACH=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

q() {
  printf "%q" "$1"
}

REMOTE_CMD="if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx koopa-kingdom; then docker exec -u admin koopa-kingdom bash -lc 'cd /autonav/scripts/real_robot_calibration && ./stop_on_robot.sh'; elif [ -x $(q "$REMOTE_SUITE_DIR")/stop_on_robot.sh ]; then $(q "$REMOTE_SUITE_DIR")/stop_on_robot.sh; else echo 'stop_on_robot.sh not found'; fi"
ssh -tt "$ROBOT" "$REMOTE_CMD"

if [ "$ATTACH" -eq 1 ]; then
  ssh -tt "$ROBOT" "tmux attach -t $(q "$SESSION")"
fi
