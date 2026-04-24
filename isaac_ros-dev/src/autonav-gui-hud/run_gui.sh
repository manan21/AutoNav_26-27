#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
# Requires: sudo ./install.sh to have been run first.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
python3 -m autonav_gui_hud.hud_node "$@"
