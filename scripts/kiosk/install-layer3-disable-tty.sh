#!/usr/bin/env bash
#
# Layer 3: disable TTY switching (Ctrl+Alt+F1..F6 -> no-op).
# See GUI_SAFETY_PLAN.md section 6.
#
# DANGEROUS: after this layer is applied and the next reboot lands,
# SSH is the only recovery path. Do not run until you have:
#   - sshd enabled and active
#   - your SSH key in ~/.ssh/authorized_keys on the Jetson
#   - tested SSH login from at least one other machine
#
# scan.sh checks all three.

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG="$HERE/config"

confirm() {
    local prompt="$1"
    if [ "${ASSUME_YES:-0}" = "1" ]; then return 0; fi
    read -r -p "$prompt [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]]
}

echo "=== Layer 3: disable TTY switching ==="
cat <<'EOF'

After this layer + reboot, the only way to recover the Jetson if the
GUI session breaks is SSH. Please confirm:

  - ssh service is enabled (systemctl is-enabled ssh)
  - your authorized_keys is on the Jetson and tested
  - you have a second device able to SSH in
  - you have run scan.sh and reviewed its SSH section

EOF
confirm "All of the above are true. Proceed?" || { echo "aborted."; exit 1; }

# --- logind drop-in ---
echo "[1/2] Installing /etc/systemd/logind.conf.d/10-kiosk.conf"
sudo install -d -m 0755 /etc/systemd/logind.conf.d
sudo install -m 0644 "$CFG/logind.conf.d/10-kiosk.conf" /etc/systemd/logind.conf.d/10-kiosk.conf

# Note: a daemon-reload is not enough for logind to drop existing
# getty@tty[2-6] units; those need to be masked or the system needs
# a reboot. We mask them explicitly so the change is effective without
# requiring an immediate reboot.
echo "[2/2] Masking getty@tty2..tty6"
for n in 2 3 4 5 6; do
    sudo systemctl stop "getty@tty${n}.service" 2>/dev/null || true
    sudo systemctl mask "getty@tty${n}.service" 2>/dev/null || true
done

# Optional: remove console=tty1 from the kernel cmdline so the kernel
# does not even hand tty1 to userspace as a console. Skipped here
# because the Jetson uses extlinux and the cmdline lives in
# /boot/extlinux/extlinux.conf which the user should edit by hand
# after reading the plan. The two changes above already kill
# Ctrl+Alt+Fn switching for an attacker at the keyboard.
echo
echo "Optional manual step: edit /boot/extlinux/extlinux.conf and"
echo "remove any 'console=tty1' from APPEND lines. Keep console=ttyS0"
echo "if you use serial debugging. Reboot to take effect."
echo

cat <<EOF

Layer 3 install complete.

Test after next reboot:
  - Ctrl+Alt+F2 should switch to nothing.
  - \`sudo chvt 2\` should fail.
  - \`ls /dev/tty[2-6]\` should show those devices absent or unowned.
  - SSH should still work.

Rollback:
  - sudo rm /etc/systemd/logind.conf.d/10-kiosk.conf
  - sudo systemctl unmask getty@tty2.service ... getty@tty6.service
  - reboot

EOF
