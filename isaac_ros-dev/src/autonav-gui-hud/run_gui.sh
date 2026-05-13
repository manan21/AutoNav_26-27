#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
# Requires: sudo ./install.sh to have been run first.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2 so rclpy is available for live mode
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

AUTONAV_FASTDDS_PROFILE_FILE_DEFAULT="${REPO_ROOT}/env/docker/fastdds_udp.xml"
source "${REPO_ROOT}/env/docker/dds-env.sh"

# Force system Qt5 plugins so PyQt5/matplotlib render correctly
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms
export DISPLAY="${DISPLAY:-:0}"

python3 -m autonav_gui_hud.hud_node "$@"
