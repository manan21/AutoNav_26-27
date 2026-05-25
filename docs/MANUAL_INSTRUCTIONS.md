# Manual Operation Instructions

Three tiers of control, in order of preference. Use the highest tier that's available — fall back only when the tier above it is broken.

| Tier | Path | Use when |
|---|---|---|
| **1. GUI** | Robot's on-screen HUD launches and monitors the full stack | Normal operation — this is the recommended path. See also [`LAUNCH_STACK.md`](./LAUNCH_STACK.md). |
| **2. Manual ROS2** | SSH into the Jetson, `ros2 launch control control_dev.launch.py`, drive with the Xbox controller | GUI is unavailable but the Jetson + container are working |
| **3. Legacy direct** | Laptop → USB serial → motor controller, bypassing ROS2 entirely (`scripts/tempcontrol/`) | ROS2 / the Jetson container is unavailable. Works on either **Shogi** or **Bowser**. |

---

## Tier 1: GUI operation (recommended)

The GUI is the recommended way to bring up the full sensor stack and run the robot autonomously. It launches and monitors every device (camera, LiDAR, SLAM, NAV2, GPS, etc.) for you — no manual `ros2 launch` calls needed.

1. Turn the Shogi on.
2. Make sure to connect the motor controller USB, then the Arduino USB.
3. Connect the USB-C to the Jetson and the USB-A to the laptop.
4. On the laptop, press `Ctrl + Alt + T` to open a terminal.
5. Turn on the Xbox joystick by pressing the middle button (light should be white and steady).
6. **Terminal 1 — start the container:**
    * `ssh jetson`
    * `./AutoNav_25-26/env/docker/run-container.sh`
    * **WAIT UNTIL THE CONTAINER BOOTS UP**
7. **On the robot itself (small screen) — start the GUI**:
    * Sign in on the Jetson directly (using its attached keyboard/screen).
    * Double-tap (or double-click) the GUI terminal command shortcut on the Desktop.
8. The GUI window will appear. From here, everything is point-and-click — no extra commands.

### Using the GUI

* **Connect to Container** — When a container is up, click this so the GUI can `docker exec` ROS2 commands inside it. Without this, the launch buttons can't fire.
* **Launch panel** — toggle subsystems on/off in queue order: Pre-SLAM, Camera, Lidar, GPS, PCA DETECT, CAMERA LINE DETECT, LIDAR LINE DETECT (opt-in), SLAM, NAV2, Power PCB. See [`LAUNCH_STACK.md`](./LAUNCH_STACK.md) for the full order and dependency notes.
* **Status dots** — gray = off, yellow = starting, green = ready.
* **Terminal viewer** — click any device to stream its live stdout/stderr.
* **Sensor plots** — live odom, IMU, GPS, and costmap previews.

[Link to usage guide](https://virginiatech.sharepoint.com/:p:/r/sites/IDC2024-2025/AutoNav/Shared%20Documents/AutoNav%202025%20-%202026/Assembly%20Manuals,%20Datasheets,%20etc/How%20To%20Use%20the%20AutoNav%20GUI%20on%20the%20robot.pptx?d=wd1ee80a78f684e6ca5040fde4d8fab1a&csf=1&web=1&e=ulmUEs)

### Engaging autonomy

* Once the launch panel is all green, or all processes are up on separate terminal windows, set the RViz frame to `map`.
* The e-stop must be **rotated** to disengage — restart the robot afterward, or it won't move.
* Press **X** on the Xbox controller to enter autonomous mode.
* The robot follows any waypoints automatically; the controller still works for **B** (E-Stop) and **A** (automated testing) at any time.

---

## Tier 2: Manual ROS2 operation

Use this when the GUI is unavailable but the Jetson and container are working — you bring up only the control node and drive with the Xbox controller.

1. Turn the Shogi on.
2. Make sure to connect the motor controller USB, then the Arduino USB.
3. Connect the USB-C to the Jetson and the USB-A to the laptop.
4. On the laptop, press `Ctrl + Alt + T` to open the terminal.
5. Turn on the Xbox joystick by pressing the middle button (light should be white and steady).
6. Now paste the following commands one by one:
    * `ssh jetson`
    * `./AutoNav_25-26/env/docker/run-container.sh`
    * **WAIT UNTIL THE CONTAINER BOOTS UP**
    * `ros2 launch control control_dev.launch.py`
7. Wait for the node to boot up (Arduino connected, Motor Controller connected, Joystick connected).
8. You are ready to operate the robot.

### Controls

* **Left joystick** — left side of the robot
* **Right joystick** — right side of the robot
* **RB** (right shoulder bumper) — incrementally increase speed
* **LB** (left shoulder bumper) — incrementally decrease speed
* **X** — starts Autonomous mode
* **B** — E-Stop (have to reboot the robot if pressed)
* **A** — starts automated testing / data acquisition

> **Note:** Speeds have been greatly slowed down on the bumpers, but still be cautious about the robot speed.

---

## Tier 3: Legacy direct control

> ⚠️ Use this only when ROS2 / the Jetson container is unavailable. It talks directly to the motor controller over serial from a laptop, bypassing the rest of the stack. Works on either **Shogi** or **Bowser**. This script has speed differences from the normal motor controller path — use care, and **always have someone manning the physical e-stop**.

The scripts live under `scripts/tempcontrol/`.

### One-time install

Requirements: **Python 3.10+** (earlier versions might work). Check with `python3 --version`.

From the top of the repo, open a command prompt and `cd scripts/tempcontrol`. Then:

```bash
python3 -m venv .venv
```

This creates a virtual environment to hold and isolate dependencies.

Activate the environment:

```bash
# Linux/Mac:
source .venv/bin/activate
# Windows:
.venv\Scripts\activate
```

You can leave the virtual environment at any time by running `deactivate`.

Finally, install the requirements:

```bash
pip install -r requirements.txt
```

### Running

Activate the environment if it isn't already (`.venv` should appear in your prompt), then pick a connection mode:

#### Wired (single computer)

```bash
python3 drivebowser.py PORT
```

Where `PORT` is the serial port for the motor controller. The application prints a usage guide in the terminal at launch.

#### Bluetooth (two computers)

Both computers need to have gone through the one-time install above.

On the **remote** computer (the one you're holding):

```bash
python3 bowser_remote.py PORT
```

Where `PORT` is the serial port for the Bluetooth line.

On the **robot-side** computer (sitting on the robot, talking to the motor):

```bash
python3 bowser_receiver.py BLUETOOTH_PORT MOTOR_PORT
```

With the Bluetooth serial port and the motor serial port, in that order.

### Speed step-size

A `step_size` parameter controls speed resolution. Default `10`. It divides the motor's native speed resolution of `1000`, so:

- **MAX SPEED OUT OF THE BOX IS 100** (= 1000 / 10).
- The `e` (speed up) and `q` (speed down) commands show speed in step-size units, not raw motor units.

The divider exists to (a) reduce wear on the `e`/`q` keys and (b) make per-press speed changes more meaningful. If you need finer resolution, change `step_size` in source — there's no menu option for it currently.
