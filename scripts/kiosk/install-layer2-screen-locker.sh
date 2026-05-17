#!/usr/bin/env bash
#
# Layer 2: real screen locker (xscreensaver) + PAM rate limit.
# See GUI_SAFETY_PLAN.md section 5.
#
# Run on the Jetson as a normal admin user. Requires sudo.

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG="$HERE/config"
KIOSK_USER="autonav-kiosk"
KIOSK_HOME="/home/$KIOSK_USER"

confirm() {
    local prompt="$1"
    if [ "${ASSUME_YES:-0}" = "1" ]; then return 0; fi
    read -r -p "$prompt [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]]
}

echo "=== Layer 2: real screen locker ==="

if ! id "$KIOSK_USER" >/dev/null 2>&1; then
    echo "ERROR: $KIOSK_USER does not exist. Run install-layer1-kiosk-wm.sh first."
    exit 1
fi

if ! dpkg -s xscreensaver >/dev/null 2>&1; then
    echo "[1/3] Installing xscreensaver"
    sudo apt-get update
    sudo apt-get install -y xscreensaver
else
    echo "[1/3] xscreensaver already installed"
fi

# --- xscreensaver config for the kiosk user ---
echo "[2/3] Installing ~/.xscreensaver for $KIOSK_USER"
sudo install -o "$KIOSK_USER" -g "$KIOSK_USER" -m 0644 \
    "$CFG/xscreensaver" "$KIOSK_HOME/.xscreensaver"

# --- PAM rate limit ---
echo "[3/3] Installing /etc/pam.d/xscreensaver with pam_faillock"
if [ -e /etc/pam.d/xscreensaver ]; then
    ts="$(date +%Y%m%d-%H%M%S)"
    sudo cp /etc/pam.d/xscreensaver "/etc/pam.d/xscreensaver.bak.$ts"
    echo "  existing file backed up to /etc/pam.d/xscreensaver.bak.$ts"
fi
sudo install -m 0644 "$CFG/pam.d/xscreensaver" /etc/pam.d/xscreensaver

# Sanity-check that pam_faillock.so is present on this distro.
if ! find /lib*/security /usr/lib*/security 2>/dev/null | grep -q pam_faillock.so; then
    echo "WARN: pam_faillock.so not found in the standard PAM module dirs."
    echo "      The screen locker will likely fail to authenticate at all."
    echo "      Install the libpam-modules-bin / pam package providing it."
fi

cat <<EOF

Layer 2 install complete.

Test plan:
  1. From the GUI (still running as $KIOSK_USER after Layer 1 reboot):
     - Press Ctrl+Shift+L. The screen should blank and xscreensaver's
       unlock dialog should appear.
     - Type the $KIOSK_USER password and press Enter -> unlock.
  2. Trigger PAM rate limit: type a wrong password 3 times.
     - The unlock prompt should refuse further attempts for 15 minutes.
     - From SSH: \`sudo faillock --user $KIOSK_USER --reset\` clears it.

The in-app PyQt lock overlay has already been removed from hud_node.py;
Ctrl+Shift+L now calls xscreensaver-command -lock from inside the GUI
as a fallback (the WM keybind does the same).

EOF
