# GUI Security — Handoff to Next Team

> The OS-level kiosk lockdown was designed and scripted on the
> `improve/gui-security` branch but **was not deployed** before the
> 2026 competition. A Qt-level bandaid is the only safeguard
> currently active. This doc tells the next team what's already
> done, what's left, and how to finish it safely.

## What's currently active (the bandaid)

A single in-app guard in `isaac_ros-dev/src/autonav-gui-hud/autonav_gui_hud/hud_node.py`:

- `HudWindow.closeEvent` ignores every close attempt unless `self._authorized_close` is `True`.
- The flag is only flipped by `_confirm_exit_password` — the PAM-gated **Quit GUI** button — immediately before `self.close()`.
- This blocks **ALT+F4**, the WM close button, and anything else that goes through Qt's close path.

### What the bandaid does NOT block

Out of Qt's reach by design; documented in the `closeEvent` comment:

| Escape vector | Why Qt can't block it |
|---|---|
| Super key, ALT+Tab | Window manager / desktop environment |
| 3-finger swipe, touchpad gestures | libinput / gnome-shell, before Qt |
| Ctrl+Alt+F1..F6 (TTY switch) | Kernel / systemd-logind |
| Ctrl+Alt+Backspace (X zap) | Xorg server |
| `kill <pid>` from terminal or SSH | Bypasses Qt entirely |
| System shutdown SIGTERM | Bypasses Qt entirely |

If any of those matter for your threat model, the OS-level kiosk layers below are the real fix.

## What's already designed but NOT deployed

The branch ships a four-layer kiosk hardening plan in `scripts/kiosk/`:

| Layer | Script | What it does |
|---|---|---|
| 0 | `install-layer0-gui.sh` | Copies the GUI launcher + Python package into `/opt/autonav/` so a separate kiosk user (no group access to admin home) can launch it. |
| 1 | `install-layer1-kiosk-wm.sh` | Installs lightdm + openbox + python3-pam, creates `autonav-kiosk` user, autologin into an Openbox session with a stripped keymap that disables ALT+F4 / ALT+Tab / Super / Ctrl+Alt+T / Ctrl+Alt+Backspace. **Requires reboot.** |
| 2 | `install-layer2-screen-locker.sh` | xscreensaver locker bound to Ctrl+Shift+L with a PAM rate limit (3 fails → 15 min lockout). |
| 3 | `install-layer3-disable-tty.sh` | `logind.conf.d` drop-in + masked gettys. Ctrl+Alt+F1..F6 stop working. **Requires reboot.** |

Each layer is **independently testable and independently reversible**, but the safety of "reversible" depends on having SSH still working — see Recovery below.

Full overview lives in `scripts/kiosk/README.md`. Read that first.

## Why it wasn't deployed for 2026

The kiosk's biggest risk is Layer 1, which **swaps the display manager** from the L4T-default `gdm3` to `lightdm`. If lightdm's autologin or the Openbox session is misconfigured, the Jetson boots to a black screen and recovery is SSH-only. With three days to go before competition, this risk was deemed too high for a daily-driver robot. The bandaid was added to cover the most common operator escape (ALT+F4) without touching the DM.

## How to finish it (recommended path)

The plan assumes you're at the Jetson with a second device that can SSH in for recovery.

### 1. Pre-flight (read-only, zero risk)

```bash
cd ~/AutoNav_25-26
./scripts/kiosk/scan.sh
```

Confirms display server, current DM, SSH state, TTY config, package presence. No state is changed.

### 2. Layer 0 (low risk, easy undo)

```bash
./scripts/kiosk/install-layer0-gui.sh
```

Installs `/opt/autonav/` with the GUI files and a self-contained launcher. Undo with `sudo rm -rf /opt/autonav`.

### 3. Test the WM lockdown WITHOUT touching the production DM (recommended)

Before committing to Layer 1, sanity-check Openbox's keymap in an embedded X server:

```bash
sudo apt install xserver-xephyr openbox openbox-session
Xephyr -screen 1920x720 :2 &
DISPLAY=:2 openbox-session &
DISPLAY=:2 /opt/autonav/run-gui.sh
```

ALT+F4 / ALT+Tab inside the Xephyr window should be blocked by Openbox's stripped keymap (see `scripts/kiosk/config/openbox/rc.xml`). Close the Xephyr window to tear it all down. **No DM change, no autologin, no reboot.**

### 4. Layer 1 (HIGH risk — DM swap + reboot)

Only after step 3 passes:

```bash
./scripts/kiosk/install-layer1-kiosk-wm.sh
sudo reboot
```

**Have a second device with SSH access ready before rebooting.** If lightdm fails, your only path back is `ssh jetson` → `sudo systemctl disable lightdm && sudo systemctl enable gdm3 && sudo reboot`.

### 5. Layer 2 (low risk)

```bash
./scripts/kiosk/install-layer2-screen-locker.sh
```

Just config + PAM. No reboot.

### 6. Layer 3 (HIGH risk if SSH ever fails afterwards — reboot)

```bash
./scripts/kiosk/install-layer3-disable-tty.sh
sudo reboot
```

After this, Ctrl+Alt+F1..F6 do nothing. If X also dies and SSH is down, recovery requires physically pulling the disk or single-user mode via bootloader. Don't run this until SSH recovery has been verified from **two** independent devices.

## After deployment: revisit the bandaid

The Qt-level `_authorized_close` guard becomes redundant once Layer 1 strips ALT+F4 from the WM keymap. You can either:

- **Leave it in place.** Defense in depth; harmless if the kiosk layers all work.
- **Remove it.** Delete the `_authorized_close` flag and the guard at the top of `closeEvent`. The PAM-gated Quit GUI button still calls `self.close()` and tears down cleanly; the difference is that any future close path (system shutdown, unexpected WM event) won't be silently ignored.

If you remove it, make sure operator workflows don't depend on ALT+F4 being a no-op.

## Recovery cheat sheet

| Failure | Recovery |
|---|---|
| Layer 1 left lightdm broken, boots to black screen | `ssh jetson`, `sudo systemctl disable lightdm && sudo systemctl enable gdm3 && sudo reboot` |
| Layer 2 locked the kiosk user out (3 failed passwords → 15 min lockout) | `ssh jetson`, `sudo faillock --user autonav-kiosk --reset` |
| Layer 3 disabled TTYs and SSH is also down | Boot to single-user mode via bootloader, or pull the SD/eMMC. Don't get here. |
| Bandaid prevents legitimate GUI close (PAM dialog stuck, no keyboard) | `ssh jetson`, `pkill -f autonav_gui_hud` |

## Related files

- `scripts/kiosk/README.md` — full design rationale and rollout order
- `scripts/kiosk/scan.sh` — pre-flight, read-only
- `scripts/kiosk/install-layer0-gui.sh` through `install-layer3-disable-tty.sh`
- `scripts/kiosk/config/` — openbox keymap, xscreensaver, PAM, xorg, logind, lightdm configs
- `isaac_ros-dev/src/autonav-gui-hud/autonav_gui_hud/hud_node.py` — `_confirm_exit_password`, `closeEvent`, `_authorized_close`
