### Manual operation of the Shogi:

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

#### Controls:
* **Left joystick** — left side of the robot
* **Right joystick** — right side of the robot
* **RB** (right shoulder bumper) — incrementally increase speed
* **LB** (left shoulder bumper) — incrementally decrease speed
* **X** — starts Autonomous mode
* **B** — E-Stop (have to reboot the robot if pressed)
* **A** — starts automated testing / data acquisition

> **Note:** Speeds have been greatly slowed down on the bumpers, but still be cautious about the robot speed.

---

### GUI operation of the Shogi:

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

#### Using the GUI:
* **Connect to Container** — When a container is up, click this so the GUI can `docker exec` ROS2 commands inside it. Without this, the launch buttons can't fire.
* **Launch panel** — toggle subsystems on/off in queue order: Pre-SLAM, Camera, Lidar, SLAM, DETECT, NAV2, GPS, Power PCB.
* **Status dots** — gray = off, yellow = starting, green = ready.
* **Terminal viewer** — click any device to stream its live stdout/stderr.
* **Sensor plots** — live odom, IMU, GPS, and costmap previews.

[Link to usage guide](https://virginiatech.sharepoint.com/:p:/r/sites/IDC2024-2025/AutoNav/Shared%20Documents/AutoNav%202025%20-%202026/Assembly%20Manuals,%20Datasheets,%20etc/How%20To%20Use%20the%20AutoNav%20GUI%20on%20the%20robot.pptx?d=wd1ee80a78f684e6ca5040fde4d8fab1a&csf=1&web=1&e=ulmUEs)

#### Engaging autonomy:
* Once the launch panel is all green, or all processes are up on separate terminal windows, set the RViz frame to `map`.
* The e-stop must be **rotated** to disengage — restart the robot afterward, or it won't move.
* Press **X** on the Xbox controller to enter autonomous mode.
* The robot follows any waypoints automatically; the controller still works for **B** (E-Stop) and **A** (automated testing) at any time.
