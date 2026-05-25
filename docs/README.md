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
- **Status dots** — red = off, yellow = starting, green = ready (each script emits `[GUI_READY] <Label>` after a fixed 5 s pacing timer).
- **Terminal viewer** — click any device to stream its live stdout/stderr.
- **Sensor plots** — live odom, IMU, GPS, costmap previews.

## Navigating
- Set the RViz frame to `map`.
- The e-stop must be **rotated** to disengage — restart the robot afterward, or it won't move.
- Press `X` on the Xbox controller to enter autonomous mode.

# Debugging the LiDAR
It's on `eno1`. The topic is `/scan_fullframe` (some legacy code still uses `/scan` — likely needs unifying).

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

# Helpful Background Documentation
- [TF transforms](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html) — `map → odom → base_link → sensor` is the canonical chain.
- [URDF + Robot State Publisher](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html) — our `robot_state_publisher` is configured in `core_bringup.launch.py`.
- [Quick Nav2 overview](https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2)
- [Detailed Nav2 docs](https://docs.nav2.org/concepts/index.html)
