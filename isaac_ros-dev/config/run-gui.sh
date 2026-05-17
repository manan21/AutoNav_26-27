#!/usr/bin/env bash
# Launch the AutoNav GUI HUD natively on the Jetson.
#
# Also brings up an xscreensaver daemon so Ctrl+Shift+L (intercepted
# inside the GUI and forwarded to `xscreensaver-command -lock`)
# actually locks the screen even outside the full kiosk session.
# Pure best-effort: if xscreensaver isn't installed the GUI still
# launches, the lock hotkey just becomes a no-op. See
# scripts/kiosk/README.md for the full kiosk-session setup.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GUI_DIR="$SCRIPT_DIR/../src/autonav-gui-hud"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
KIOSK_XSCREENSAVER="$REPO_ROOT/scripts/kiosk/config/xscreensaver"

# Source ROS2 so rclpy is available for live mode
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Match the container's ROS domain
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Force system Qt5 plugins so PyQt5/matplotlib render correctly
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms
export DISPLAY="${DISPLAY:-:0}"

# Bring up the OS screen locker. pgrep makes this idempotent so a
# re-launch of run-gui.sh doesn't spawn a second daemon.
if command -v xscreensaver >/dev/null 2>&1; then
    # First-run: drop in the kiosk locker config (manual lock only,
    # 30s passwd timeout). User can override by editing ~/.xscreensaver.
    if [ ! -f "$HOME/.xscreensaver" ] && [ -r "$KIOSK_XSCREENSAVER" ]; then
        cp "$KIOSK_XSCREENSAVER" "$HOME/.xscreensaver"
    fi
    if ! pgrep -x xscreensaver >/dev/null 2>&1; then
        xscreensaver -no-splash >/dev/null 2>&1 &
        disown 2>/dev/null || true
    fi
else
    echo "[run-gui] xscreensaver not installed; Ctrl+Shift+L will not lock." >&2
fi

cd "$GUI_DIR"
python3 -m autonav_gui_hud.hud_node "$@"
