# AutoNav kiosk hardening

System-side implementation of the three-layer plan in
`GUI_SAFETY_PLAN.md` (lives in the GUI sandbox). Code-side change —
removal of the in-app PyQt lock overlay — is already in
`isaac_ros-dev/src/autonav-gui-hud/autonav_gui_hud/hud_node.py`.

The defense is layered:

| Layer | Mechanism                                 | What it stops                                                                            |
|------:|-------------------------------------------|------------------------------------------------------------------------------------------|
| 0     | Install GUI launcher + package to `/opt/autonav/` | Kiosk user can't read admin's home; gives a system-readable, kiosk-callable entry point |
| 1     | Openbox kiosk session + autologin         | Alt+Tab / Alt+F4 / Super / Ctrl+Alt+T / Ctrl+Alt+Backspace and any other "escape the GUI" |
| 2     | xscreensaver locker bound to Ctrl+Shift+L | Real input grab + PAM rate limit (3 fails → 15 min lockout)                              |
| 3     | systemd-logind drop-in + masked gettys    | Ctrl+Alt+F1..F6 cannot drop to a text console                                            |

## Files

```
scripts/kiosk/
├── README.md                       — this file
├── scan.sh                         — pre-flight scan (read-only)
├── install-layer0-gui.sh           — install GUI launcher + package into /opt/autonav/
├── install-layer1-kiosk-wm.sh
├── install-layer2-screen-locker.sh
├── install-layer3-disable-tty.sh
└── config/
    ├── openbox/
    │   ├── rc.xml                  — Openbox keymap (only Ctrl+Shift+L)
    │   └── autostart               — start xscreensaver daemon, exec GUI
    ├── xscreensaver                — locker config (manual lock only, 5min passwd timeout)
    ├── pam.d/xscreensaver          — PAM stack with pam_faillock deny=3 unlock=900
    ├── xorg.conf.d/99-no-zap.conf  — disable Ctrl+Alt+Backspace
    ├── logind.conf.d/10-kiosk.conf — NAutoVTs=0, ReserveVT=0
    └── lightdm.conf.d/50-autologin.conf — autologin autonav-kiosk into openbox
```

## Rollout order

Each layer is independently testable and independently reversible.
Run scan first, then the layers strictly in order; do not skip ahead.

```bash
# 1. Read-only pre-flight. Confirms display server, DM, SSH state.
./scripts/kiosk/scan.sh

# 2. Install GUI launcher + Python package into /opt/autonav/ so the
#    autonav-kiosk user (separate UID, no group access to admin home)
#    can launch it. Idempotent; rerun after pulling new GUI code.
./scripts/kiosk/install-layer0-gui.sh

# 3. Kiosk WM + autologin. Pulls python3-pam so the Quit GUI button's
#    PAM gate works. Reboot at the end.
./scripts/kiosk/install-layer1-kiosk-wm.sh
sudo reboot

# 4. Real screen locker + PAM rate limit. No reboot required.
./scripts/kiosk/install-layer2-screen-locker.sh

# 5. Disable TTY switching. ONLY after SSH recovery is confirmed
#    from a second device. Reboot at the end.
./scripts/kiosk/install-layer3-disable-tty.sh
sudo reboot
```

`ASSUME_YES=1` skips interactive confirmations in any layer script.

`AUTONAV_RUN_GUI=/path/to/run-gui.sh` overrides the GUI launcher path
in Layer 1 (default: `/opt/autonav/run-gui.sh`).

## Recovery

- **Layer 1 broke the desktop session.** SSH in as your normal admin
  user and `sudo systemctl disable lightdm && sudo reboot`. The
  Jetson will boot to a getty; from there, fix Openbox/autologin and
  re-enable lightdm.
- **Layer 2 locked the kiosk user out** (3 wrong passwords). SSH in
  and `sudo faillock --user autonav-kiosk --reset`.
- **Layer 3 left you headless with no SSH.** This shouldn't happen if
  Layer 3 prerequisites were checked (`scan.sh`). If it does, you'll
  need a USB keyboard + serial console (ttyS0 is preserved). Recovery
  from there: edit `/etc/systemd/logind.conf.d/10-kiosk.conf` to
  remove the `NAutoVTs=0` line, `systemctl unmask getty@tty2`,
  reboot.

## Hash of the in-app lock removal

The hud_node.py edits are in this same commit and are:

- Deleted: `_screen_locked`, `_lock_password`, `_lock_overlay`,
  `_lock_password_input`, `_lock_password_visible`,
  `_lock_hint_label`, `_lock_status_label` (all in `__init__`).
- Deleted: `_build_lock_overlay`, `_toggle_screen_lock`,
  `_focus_lock_password`, `_show_lock_password_input`,
  `_try_unlock`, `_unlock_screen` methods.
- Replaced: `_lock_screen` now just `subprocess.Popen(["xscreensaver-command", "-lock"])`.
- Simplified: `eventFilter` no longer has any "is the screen locked"
  state machine; it intercepts Ctrl+Shift+L and forwards sensor
  clicks. `resizeEvent` no longer resizes a lock overlay.
