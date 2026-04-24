#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GUI_DIR="$SCRIPT_DIR/../src/autonav-gui-hud"

export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms
export DISPLAY="${DISPLAY:-:0}"

cd "$GUI_DIR"
python3 -m autonav_gui_hud.hud_node "$@"
