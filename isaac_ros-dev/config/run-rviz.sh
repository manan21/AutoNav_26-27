#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f /.dockerenv ]] || grep -qaE '/docker/|/containerd/' /proc/1/cgroup 2>/dev/null; then
    exec "${SCRIPT_DIR}/run-rviz-jetson.sh" "$@"
fi

exec "${SCRIPT_DIR}/run-rviz-laptop.sh" "$@"
