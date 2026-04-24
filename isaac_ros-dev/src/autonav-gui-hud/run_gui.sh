#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
# Requires: ./install.sh to have been run first.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ ! -d "$SCRIPT_DIR/.venv" ]; then
    echo "Virtual environment not found. Run ./install.sh first."
    exit 1
fi

source "$SCRIPT_DIR/.venv/bin/activate"
python -m autonav_gui_hud.hud_node "$@"
