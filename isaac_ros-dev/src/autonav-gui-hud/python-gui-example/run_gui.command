#!/usr/bin/env bash
# Double-click this file in Finder to launch the AutoNav GUI HUD
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/../.venv/bin/activate"
python "$SCRIPT_DIR/hud_node.py" "$@"
