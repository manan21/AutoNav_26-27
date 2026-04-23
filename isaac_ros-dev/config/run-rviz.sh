#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_PATH="${AUTONAV_RVIZ_CONFIG:-${SCRIPT_DIR}/../src/sim/config/view_bot.rviz}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

for passthrough_var in RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; do
    if [[ -n "${!passthrough_var:-}" ]]; then
        export "${passthrough_var}=${!passthrough_var}"
    fi
done

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "ERROR: rviz2 is not installed or not on PATH."
    echo "Install ROS 2 Humble RViz on the laptop, then re-run this script."
    exit 1
fi

echo "run-rviz.sh: launching native RViz"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
for passthrough_var in RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; do
    if [[ -n "${!passthrough_var:-}" ]]; then
        echo "${passthrough_var}=${!passthrough_var}"
    fi
done

rviz2 -d "$NAV_PATH"
