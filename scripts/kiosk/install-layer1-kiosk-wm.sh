#!/usr/bin/env bash
#
# Layer 1: kiosk window manager (Openbox) + dedicated kiosk user.
# See GUI_SAFETY_PLAN.md section 4.
#
# Run on the Jetson as a normal admin user. Uses sudo for system files.
# Idempotent: re-running should converge to the same state.

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG="$HERE/config"
KIOSK_USER="autonav-kiosk"
KIOSK_HOME="/home/$KIOSK_USER"
RUN_GUI="${AUTONAV_RUN_GUI:-/opt/autonav/run-gui.sh}"

confirm() {
    local prompt="$1"
    if [ "${ASSUME_YES:-0}" = "1" ]; then return 0; fi
    read -r -p "$prompt [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]]
}

echo "=== Layer 1: kiosk WM ==="
echo "kiosk user : $KIOSK_USER"
echo "GUI script : $RUN_GUI"
echo "configs    : $CFG"
echo

if [ ! -x "$RUN_GUI" ]; then
    echo "WARN: $RUN_GUI is not executable or does not exist."
    echo "      Set AUTONAV_RUN_GUI=/path/to/run-gui.sh and re-run, or"
    echo "      install the GUI launcher there before applying this layer."
    confirm "Continue anyway?" || exit 1
fi

# --- packages ---
# python3-pam: Debian C binding (`import PAM`) used by the GUI's
# Quit-button PAM gate (HudWindow._confirm_exit_password). Bundling
# it here so the Quit dialog works on a fresh kiosk install — without
# it the dialog falls through to a warning and the operator cannot
# quit the GUI from inside the locked-down session.
echo "[1/6] Installing packages: openbox openbox-session xscreensaver lightdm unclutter xinit python3-pam"
sudo apt-get update
sudo apt-get install -y openbox openbox-session xscreensaver lightdm unclutter xinit python3-pam

# --- kiosk user ---
echo "[2/6] Ensuring kiosk user exists"
if ! id "$KIOSK_USER" >/dev/null 2>&1; then
    # Interactive shell is /usr/sbin/nologin: the password is for PAM
    # (xscreensaver unlock) only; the user can never log in via SSH or TTY.
    sudo useradd -m -s /usr/sbin/nologin "$KIOSK_USER"
    echo "Created $KIOSK_USER. Set its password (used by xscreensaver unlock):"
    sudo passwd "$KIOSK_USER"
else
    echo "  $KIOSK_USER already exists; not changing password."
    confirm "Reset password now?" && sudo passwd "$KIOSK_USER" || true
fi

# Workaround: lightdm refuses to auto-login a user whose shell is nologin.
# Add /usr/sbin/nologin to /etc/shells so lightdm treats it as valid for
# PAM session creation. This does NOT make nologin into an interactive
# shell; pam_shells just stops rejecting the login on the basis of the
# shell field.
if ! grep -qx '/usr/sbin/nologin' /etc/shells; then
    echo "[2/6] Allowing nologin in /etc/shells for lightdm"
    echo '/usr/sbin/nologin' | sudo tee -a /etc/shells >/dev/null
fi

# --- openbox config ---
echo "[3/6] Installing Openbox config for $KIOSK_USER"
sudo install -d -o "$KIOSK_USER" -g "$KIOSK_USER" "$KIOSK_HOME/.config/openbox"
sudo install -o "$KIOSK_USER" -g "$KIOSK_USER" -m 0644 "$CFG/openbox/rc.xml"     "$KIOSK_HOME/.config/openbox/rc.xml"
# autostart needs to be executable
sudo install -o "$KIOSK_USER" -g "$KIOSK_USER" -m 0755 "$CFG/openbox/autostart"  "$KIOSK_HOME/.config/openbox/autostart"

# Substitute the GUI path into autostart if AUTONAV_RUN_GUI was overridden.
if [ "$RUN_GUI" != "/opt/autonav/run-gui.sh" ]; then
    sudo sed -i "s|/opt/autonav/run-gui.sh|$RUN_GUI|g" "$KIOSK_HOME/.config/openbox/autostart"
fi

# --- lightdm autologin ---
echo "[4/6] Installing LightDM autologin drop-in"
sudo install -d -m 0755 /etc/lightdm/lightdm.conf.d
sudo install -m 0644 "$CFG/lightdm.conf.d/50-autologin.conf" /etc/lightdm/lightdm.conf.d/50-autologin.conf

# Make sure the session file referenced by autologin-session exists.
if [ ! -r /usr/share/xsessions/openbox.desktop ]; then
    echo "WARN: /usr/share/xsessions/openbox.desktop missing. openbox-session"
    echo "      package should have installed it. Investigate before reboot."
fi

# --- DontZap ---
echo "[5/6] Installing Xorg DontZap config"
sudo install -d -m 0755 /etc/X11/xorg.conf.d
sudo install -m 0644 "$CFG/xorg.conf.d/99-no-zap.conf" /etc/X11/xorg.conf.d/99-no-zap.conf

# --- enable lightdm as the default DM ---
echo "[6/6] Enabling lightdm"
if [ -e /etc/X11/default-display-manager ]; then
    cur="$(cat /etc/X11/default-display-manager)"
    if [ "$cur" != "/usr/sbin/lightdm" ]; then
        echo "  current default-display-manager: $cur"
        confirm "Switch to lightdm?" && {
            echo /usr/sbin/lightdm | sudo tee /etc/X11/default-display-manager >/dev/null
            sudo systemctl disable "$(basename "$cur")" 2>/dev/null || true
            sudo systemctl enable lightdm
        }
    fi
else
    sudo systemctl enable lightdm
fi

cat <<EOF

Layer 1 install complete.

Next steps:
  1. Reboot. The Jetson should come up to the Openbox session and
     auto-launch the GUI as $KIOSK_USER (no password prompt).
  2. From another terminal, verify these key combos do nothing:
       - Alt+Tab          (no window switcher)
       - Alt+F4           (GUI does not close)
       - Super            (no menu)
       - Ctrl+Alt+T       (no terminal)
       - Ctrl+Alt+Backspace (no X server kill)
  3. Ctrl+Shift+L will not lock yet - that needs Layer 2.

Recovery if the kiosk session breaks: SSH in as your normal admin
user and run \`sudo systemctl disable lightdm && sudo reboot\` to
boot to a getty next time.

EOF
