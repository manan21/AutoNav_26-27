#!/usr/bin/env bash
#
# Pre-implementation scan for the kiosk hardening plan.
# See GUI_SAFETY_PLAN.md section 7.
#
# Read-only. Run as the normal admin user on the Jetson; sudo is used
# only for sshd_config inspection. No state is changed.

set -u
RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; OFF=$'\033[0m'
hr() { printf '%s\n' "----------------------------------------"; }

note() { printf "%s[note]%s %s\n" "$YEL" "$OFF" "$*"; }
ok()   { printf "%s[ok]%s   %s\n" "$GRN" "$OFF" "$*"; }
bad()  { printf "%s[FAIL]%s %s\n" "$RED" "$OFF" "$*"; }

echo "=== AutoNav kiosk pre-flight scan ==="
echo "host:    $(hostname)"
echo "user:    $(whoami)"
echo "date:    $(date -Iseconds)"
hr

echo "## Display server / WM / DM"
echo "XDG_SESSION_TYPE  = ${XDG_SESSION_TYPE:-unset}"
if command -v loginctl >/dev/null; then
    sid="$(loginctl --no-legend list-sessions 2>/dev/null | awk -v u="$(whoami)" '$3==u{print $1; exit}')"
    if [ -n "${sid:-}" ]; then
        echo "loginctl session  = $sid"
        loginctl show-session "$sid" -p Type -p Class -p State 2>/dev/null | sed 's/^/  /'
    fi
fi
echo "DM processes:"
ps -e -o pid,comm | awk '/gdm|lightdm|sddm|getty/ {print "  " $0}'
if command -v wmctrl >/dev/null; then
    echo "WM:"
    wmctrl -m 2>/dev/null | sed 's/^/  /'
else
    note "wmctrl not installed (apt install wmctrl) - skipping WM probe"
fi
hr

echo "## Current autologin / DM config"
for f in /etc/gdm3/custom.conf /etc/lightdm/lightdm.conf /etc/sddm.conf; do
    if [ -r "$f" ]; then
        echo "  $f:"
        grep -v '^\s*\(#\|$\)' "$f" 2>/dev/null | sed 's/^/    /' || true
    fi
done
if [ -d /etc/lightdm/lightdm.conf.d ]; then
    echo "  /etc/lightdm/lightdm.conf.d/:"
    ls -1 /etc/lightdm/lightdm.conf.d/ 2>/dev/null | sed 's/^/    /'
fi
systemctl status getty@tty1 --no-pager 2>/dev/null | sed -n '1,4p' | sed 's/^/  getty@tty1: /'
hr

echo "## SSH state"
if sudo -n true 2>/dev/null; then
    sudo sshd -T 2>/dev/null | grep -iE 'passwordauthentication|permitrootlogin|pubkeyauthentication' | sed 's/^/  /'
else
    note "passwordless sudo unavailable; skipping 'sudo sshd -T'."
fi
if [ -r "$HOME/.ssh/authorized_keys" ]; then
    n="$(grep -cv '^\s*\(#\|$\)' "$HOME/.ssh/authorized_keys")"
    ok "~/.ssh/authorized_keys present ($n keys)"
else
    bad "~/.ssh/authorized_keys missing - SSH key auth not set up for $(whoami)"
fi
echo "  ssh.service enabled: $(systemctl is-enabled ssh 2>/dev/null || echo unknown)"
echo "  ssh.service state:   $(systemctl is-active ssh 2>/dev/null || echo unknown)"
hr

echo "## TTY / logind state"
if [ -r /etc/systemd/logind.conf ]; then
    grep -E '^[^#]*VT' /etc/systemd/logind.conf 2>/dev/null | sed 's/^/  /' || echo "  (no VT lines set)"
fi
if [ -d /etc/systemd/logind.conf.d ]; then
    echo "  drop-ins:"
    ls -1 /etc/systemd/logind.conf.d/ 2>/dev/null | sed 's/^/    /'
fi
echo "  /dev/tty[1-6]:"
ls -1 /dev/tty[1-6] 2>/dev/null | sed 's/^/    /' || echo "    (none present)"
hr

echo "## USB inventory (informational, see plan section 8 USB allowlist)"
if command -v lsusb >/dev/null; then
    lsusb | sed 's/^/  /'
else
    note "lsusb not available"
fi
hr

echo "## GUI launch path"
for f in /opt/autonav/run-gui.sh /opt/autonav/run_gui.sh ~/run-gui.sh ~/run_gui.sh; do
    if [ -r "$f" ]; then
        echo "  found: $f"
        head -20 "$f" | sed 's/^/    /'
    fi
done
if command -v systemctl >/dev/null; then
    echo "  user units matching autonav*:"
    systemctl --user list-units 'autonav*' --no-pager 2>/dev/null | sed 's/^/    /' || true
fi
hr

echo "## Existing kiosk user check"
if id autonav-kiosk >/dev/null 2>&1; then
    ok "autonav-kiosk user already exists:"
    getent passwd autonav-kiosk | sed 's/^/  /'
else
    note "autonav-kiosk user does not exist yet (expected on first run)."
fi
hr

echo "## Packages present"
for pkg in openbox openbox-session xscreensaver lightdm unclutter wmctrl; do
    if dpkg -s "$pkg" >/dev/null 2>&1; then
        ok "$pkg installed"
    else
        note "$pkg NOT installed"
    fi
done
hr

echo "Scan complete. Review the output, then apply layers in order:"
echo "  1. install-layer1-kiosk-wm.sh"
echo "  2. install-layer2-screen-locker.sh"
echo "  3. install-layer3-disable-tty.sh   (only after SSH recovery is confirmed from 2 devices)"
