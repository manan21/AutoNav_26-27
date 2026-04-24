#!/usr/bin/env bash
# Launch the AutoNav GUI HUD
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/.venv/bin/activate"
python "$SCRIPT_DIR/python-gui-example/hud_node.py" "$@"
