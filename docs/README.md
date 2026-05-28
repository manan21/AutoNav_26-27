# Connecting to the robot
## Connecting to the Jetson from the AutoNav Laptop
First, plug the USB into the computer. This USB is used for the SSH connection to the Jetson.

```bash
ssh jetson
cd AutoNav_25-26
./env/docker/run-container.sh   # launches the container
```

## Connecting to the Jetson from your own Laptop
First, plug in the USB into your computer. Your computer might ask if it's okay for the device to connect — accept.

Add the Jetson to your SSH config (`vim ~/.ssh/config`):
```
Host jetson
      HostName 192.168.55.1
      User vtcro
      Port 22
      ForwardX11 yes
      ForwardX11Trusted yes
```

Optional — skip password prompts on every connection:
```
ssh-copy-id jetson
```

Then the same start sequence works from your own laptop:
```bash
ssh jetson
cd AutoNav_25-26
./env/docker/run-container.sh
```

## First-time GUI install (Jetson host)
Run once on the Jetson, **outside** the container:
```bash
cd ~/AutoNav_25-26/isaac_ros-dev/src/autonav-gui-hud
sudo ./install.sh
```
Installs the GUI's native deps (PyQt5, matplotlib, opencv-headless).

# Running the Robot

Once the container is up, open a **second** `ssh jetson` session (X11-forwarded if you're remote) and launch the GUI:

```bash
cd ~/AutoNav_25-26
./isaac_ros-dev/config/run-gui.sh
```

From here, everything is point-and-click — no manual `ros2 launch` needed.

## The GUI
The HUD launches and monitors every device on the robot. The container does the ROS2 work; the GUI runs natively on the Jetson and talks to the container over DDS + `docker exec`.

- **Launch buttons** — toggle each subsystem in queue order: Pre-SLAM, Camera, Lidar, GPS, PCA DETECT, CAMERA LINE DETECT, LIDAR LINE DETECT (opt-in), SLAM, NAV2, Power PCB. See `docs/LAUNCH_STACK.md` for the full order and dependency notes.
- **Status dots** — red = off, yellow = starting, green = ready (most `run-*.sh` scripts emit `[GUI_READY] <Label>` after a fixed 0.5 s pacing timer; SLAM waits longer so `map_padder` can receive the first map before NAV2 starts).
- **Terminal viewer** — click any device to stream its live stdout/stderr.
- **Sensor plots** — live odom, IMU, GPS, costmap previews.

## Navigating
- Set the RViz frame to `map`.
- The e-stop must be **rotated** to disengage — restart the robot afterward, or it won't move.
- Press `X` on the Xbox controller to enter autonomous mode.

# Debugging the LiDAR
It's on `eno1`. The topic is `/scan_fullframe` — the active pipeline uses only this topic; the vendored SICK driver references `/scan` internally but nothing in our production code subscribes to it.

If the robot can't bind to the UDP port, bring the interface up:
```bash
sudo ip addr add 192.168.0.2/24 dev eno1 && sudo ip link set eno1 up
```

# Laser Scan
Visualize `/scan_fullframe` in RViz.

# ROS2 Commands
```bash
ros2 topic list
ros2 topic hz /scan_fullframe
ros2 topic echo /scan_fullframe --once
ros2 topic info /topic
```

## View the transformation trees
```bash
ros2 run tf2_tools view_frames
```
Creates a `frames<numbers>.pdf` in the current directory.

# Rebuild Code Changes
```bash
# SSH to Jetson, pull your code
docker stop koopa-kingdom
cd AutoNav_25-26
./env/docker/run-container.sh
colcon build --symlink-install
source install/setup.bash       # only needed when adding a new package
```

If the active branch uses MPPI (`nav2_mppi_controller::MPPIController`) and the container cannot find `nav2_mppi_controller`, rebuild the dev image only:

```bash
cd ~/AutoNav_25-26
./env/docker/build_koopa-dev.sh
```

`ros-humble-nav2-mppi-controller` belongs in the higher-level dev Dockerfile so this does not require rebuilding the slow `autonav:koopa-kingdom` base image.

If fewer than 22 packages build, your submodules are likely empty — commonly `pointcloud_to_laserscan` and `zed-ros2-wrapper`. Run:
```bash
git submodule update --init
```
(We hit this once by accidentally deleting `isaac_ros-dev/`.)

# Launch RViz (locally, on your Linux machine)
We no longer run RViz inside the container — run it on your own Linux laptop with ROS2 Humble installed. As long as `ROS_DOMAIN_ID` matches the container (default `0`) and your network can reach the Jetson, topics will show up:
```bash
export ROS_DOMAIN_ID=0
rviz2
```
If topics don't appear, check the FastDDS profile at `env/docker/fastdds_udp.xml` is in use on both ends.

# Move files between container and Jetson
```bash
scp vtcro@192.168.55.1:AutoNav_25-26/isaac_ros-dev/frames.pdf /home/vtcro/Documents/
```
First address = source on the Jetson. Second = destination. Trailing slash matters.

# Other docs in this folder

- [`HUMAN-WRITTEN-README.md`](./HUMAN-WRITTEN-README.md) — high-level human-written tour of the robot and the repo.
- [`AUTONOMY_DECISION_LOG.md`](./AUTONOMY_DECISION_LOG.md) — durable record of autonomy design decisions, experiment results, failed approaches, and reversals.
- [`LAUNCH_STACK.md`](./LAUNCH_STACK.md) — order, dependencies, and pacing of the GUI launch panel buttons.
- [`MANUAL_INSTRUCTIONS.md`](./MANUAL_INSTRUCTIONS.md) — three tiers of manual control (GUI → manual ROS2 launch → laptop-direct fallback) in order of preference.
- [`SENSORS.md`](./SENSORS.md) — hardware, topics, frames, and gotchas for the SICK LiDAR, ZED camera, GPS, wheel encoders, and Power PCB.
- [`PACKAGES.md`](./PACKAGES.md) — per-package reference for every ROS2 package under `isaac_ros-dev/src/`.
- [`HYPERPARAMETER.md`](./HYPERPARAMETER.md) — tunable knobs across the autonomy stack, with safety legend and YAML inventory.
- [`IGVC_COMPETITION_RULES.md`](./IGVC_COMPETITION_RULES.md) — competition rules that should constrain robot, Nav2, perception, and simulation changes.
- [`IGVC_FORTRESS_SIM.md`](./IGVC_FORTRESS_SIM.md) — active ROS Humble + Gazebo Fortress competition simulation and runner.
- [`TROUBLESHOOTING.md`](./TROUBLESHOOTING.md) — quick triage + full chronological fix log + keyword index for past bugs.

# Helpful Background Documentation
- [TF transforms](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html) — `map → odom → base_link → sensor` is the canonical chain.
- [URDF + Robot State Publisher](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html) — our `robot_state_publisher` is configured in `core_bringup.launch.py`.
- [Quick Nav2 overview](https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2)
- [Detailed Nav2 docs](https://docs.nav2.org/concepts/index.html)
