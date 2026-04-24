#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GUI_DIR="$SCRIPT_DIR/../src/autonav-gui-hud"

# Source ROS2 so rclpy is available for live mode
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# ROS2 adds its own Qt5 libs to LD_LIBRARY_PATH which breaks
# system PyQt5 and matplotlib. Strip them out so the system
# Qt5 is used for rendering.
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd ':')
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms
export DISPLAY="${DISPLAY:-:0}"

cd "$GUI_DIR"
python3 -m autonav_gui_hud.hud_node "$@"
