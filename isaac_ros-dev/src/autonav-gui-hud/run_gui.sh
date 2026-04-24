#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
# Requires: sudo ./install.sh to have been run first.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2 so rclpy is available for live mode
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Prevent OpenCV's bundled Qt from conflicting with system PyQt5
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms
export DISPLAY="${DISPLAY:-:0}"

python3 -m autonav_gui_hud.hud_node "$@"
