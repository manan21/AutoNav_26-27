#!/usr/bin/env bash
# Install AutoNav GUI HUD dependencies natively on the Jetson.
# Run once: sudo ./install.sh
set -e

echo "=== AutoNav GUI HUD — Jetson Install ==="

apt-get update
apt-get install -y \
    python3-pyqt5 \
    python3-numpy \
    python3-pil \
    python3-pip

# Install compatible matplotlib and headless OpenCV via pip
pip3 install --upgrade matplotlib opencv-python-headless

echo ""
echo "=== Install complete ==="
echo "Launch the GUI with: ./run_gui.sh or ./config/run-gui.sh"
