#!/usr/bin/env bash
# Install AutoNav GUI HUD dependencies for native Jetson use (no container needed).
# Run once: ./install.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

echo "=== AutoNav GUI HUD — Jetson Install ==="

# Use system python3 (Jetson ships python3.8+)
PYTHON=python3
if ! command -v $PYTHON &>/dev/null; then
    echo "ERROR: python3 not found. Install with: sudo apt install python3 python3-venv python3-pip"
    exit 1
fi

# Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    $PYTHON -m venv --system-site-packages "$VENV_DIR"
fi

echo "Installing dependencies..."
source "$VENV_DIR/bin/activate"
pip install --upgrade pip
pip install -r "$SCRIPT_DIR/requirements.txt"

echo ""
echo "=== Install complete ==="
echo "Launch the GUI with: ./run_gui.sh"
