#!/usr/bin/env bash
# Install AutoNav GUI HUD dependencies natively on the Jetson.
# Run once: sudo ./install.sh
set -e

echo "=== AutoNav GUI HUD — Jetson Install ==="

apt-get update
apt-get install -y \
    python3-pyqt5 \
    python3-matplotlib \
    python3-numpy \
    python3-pil \
    python3-pip

# Use headless OpenCV to avoid Qt plugin conflicts with PyQt5
pip3 install opencv-python-headless "numpy<2"

echo ""
echo "=== Install complete ==="
echo "Launch the GUI with: ./run_gui.sh"
