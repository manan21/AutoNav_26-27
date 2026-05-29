# Real Robot RViz From A Mac

Use this runbook when a Mac needs live RViz visualization and goal control for
the physical robot over the Jetson USB link.

This is not the normal Linux-laptop RViz flow in `env/docker/REMOTE_RVIZ.md`.
macOS cannot run RViz natively, so the working setup is:

```text
Mac RealVNC Viewer
  -> localhost:5903
  -> Lima VM autonav-rviz
  -> Xvfb :3 + x11vnc + rviz2
  -> Fast DDS over robot USB bridge
  -> Jetson container ROS graph
```

## Known Working Addresses

- Jetson USB address: `192.168.55.1`
- Mac USB address: `192.168.55.100`
- RViz VM static USB address: `192.168.55.101`
- RViz VM name: `autonav-rviz`
- VNC endpoint on the Mac: `localhost:5903`
- VNC password: `autonav`
- Robot SSH target: `ssh jetson`
- Robot container: `koopa-kingdom`

If the Mac USB interface is not `en9`, replace `en9` below with the active
interface that has `192.168.55.100`.

## Fast Path

These commands assume the `autonav-rviz` Lima VM already exists and has ROS
Humble, RViz, Xvfb, x11vnc, and openbox installed.

### 1. Verify The Robot USB Link

On the Mac:

```bash
ssh jetson 'docker ps --format "table {{.Names}}\t{{.Status}}"'
ping -c 2 192.168.55.1
ifconfig en9 | grep '192.168.55.100'
```

The robot container should be running as `koopa-kingdom`.

### 2. Start The RViz VM On Both Networks

The VM needs two network paths:

- the normal Lima/host network for management and VNC port forwarding
- the robot USB bridge for DDS traffic

One-time Lima sudoers setup, if `limactl start` says the sudoers file is
missing or out of sync:

```bash
limactl sudoers > /tmp/lima_sudoers
sudo install -o root -g wheel -m 0444 /tmp/lima_sudoers /private/etc/sudoers.d/lima
```

Create the robot USB bridge if it does not exist:

```bash
limactl network list
limactl network create robotusb --mode bridged --interface en9
```

If you create or change a Lima network, regenerate the sudoers file afterward.
Lima checks that the file matches the current network list:

```bash
limactl sudoers > /tmp/lima_sudoers
sudo install -o root -g wheel -m 0444 /tmp/lima_sudoers /private/etc/sudoers.d/lima
```

`~/.lima/autonav-rviz/lima.yaml` should contain both networks:

```yaml
networks:
- lima: bridged
- lima: robotusb
```

Start the VM:

```bash
limactl start autonav-rviz
```

Assign the static USB-side address inside the VM:

```bash
limactl shell autonav-rviz bash -lc '
  sudo ip addr add 192.168.55.101/24 dev lima1 2>/dev/null || true
  sudo ip link set lima1 up
  sudo ip route add 192.168.0.0/24 via 192.168.55.1 dev lima1 2>/dev/null || true
  ping -c 2 -W 1 192.168.55.1
'
```

The `192.168.0.0/24` route matters because several robot DDS participants
advertise locators on the Jetson's LiDAR-side `192.168.0.2` interface.

### 3. Start The DDS Multicast Relay

`socket_vmnet` bridging does not reliably pass multicast discovery packets.
Unicast topic data is fine once discovery works, but Fast DDS discovery needs
help. Run this relay on both sides; it forwards only DDS discovery multicast
on `239.255.0.1:7400`.

In the VM:

```bash
limactl shell autonav-rviz bash -lc "cat >/tmp/autonav_dds_relay.py <<'PY'
import os, socket, time
GROUP = '239.255.0.1'
PORT = 7400
LOCAL = os.environ['LOCAL']
PEER = os.environ['PEER']
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except OSError:
    pass
rx.bind((GROUP, PORT))
rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(GROUP) + socket.inet_aton(LOCAL))
rx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(LOCAL))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
tx.bind((LOCAL, 0))
print('multicast relay %s -> %s:%d' % (LOCAL, PEER, PORT), flush=True)
while True:
    data, addr = rx.recvfrom(65535)
    if addr[0] == LOCAL:
        continue
    tx.sendto(data, (PEER, PORT))
    print('%.3f %d bytes from %s:%d -> %s:%d' % (time.time(), len(data), addr[0], addr[1], PEER, PORT), flush=True)
PY
pkill -f '^autonav_dds_relay' 2>/dev/null || true
nohup env LOCAL=192.168.55.101 PEER=192.168.55.1 \
  bash -c 'exec -a autonav_dds_relay python3 /tmp/autonav_dds_relay.py' \
  >/tmp/dds_multicast_relay_rviz.log 2>&1 &
pgrep -af '^autonav_dds_relay'
"
```

On the Jetson host:

```bash
ssh jetson "cat >/tmp/autonav_dds_relay.py <<'PY'
import os, socket, time
GROUP = '239.255.0.1'
PORT = 7400
LOCAL = os.environ['LOCAL']
PEER = os.environ['PEER']
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except OSError:
    pass
rx.bind((GROUP, PORT))
rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(GROUP) + socket.inet_aton(LOCAL))
rx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(LOCAL))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
tx.bind((LOCAL, 0))
print('multicast relay %s -> %s:%d' % (LOCAL, PEER, PORT), flush=True)
while True:
    data, addr = rx.recvfrom(65535)
    if addr[0] == LOCAL:
        continue
    tx.sendto(data, (PEER, PORT))
    print('%.3f %d bytes from %s:%d -> %s:%d' % (time.time(), len(data), addr[0], addr[1], PEER, PORT), flush=True)
PY
pkill -f '^autonav_dds_relay' 2>/dev/null || true
nohup env LOCAL=192.168.55.1 PEER=192.168.55.101 \
  bash -c 'exec -a autonav_dds_relay python3 /tmp/autonav_dds_relay.py' \
  >/tmp/dds_multicast_relay_rviz.log 2>&1 &
pgrep -af '^autonav_dds_relay'
"
```

### 4. Verify ROS Discovery From The VM

```bash
limactl shell autonav-rviz bash -lc '
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 daemon stop >/dev/null 2>&1 || true
  timeout 30 ros2 topic list --no-daemon | sort | grep -E "(/cloud_all_fields_fullframe|/lidar_line_detection/debug/points|/lidar_line_costmap|/global_costmap/costmap|/local_costmap/costmap|/goal_pose|/plan)$"
'
```

You should see at least:

```text
/cloud_all_fields_fullframe
/global_costmap/costmap
/goal_pose
/lidar_line_costmap
/lidar_line_detection/debug/points
/local_costmap/costmap
/plan
```

If the VM can ping `192.168.55.1` but sees only `/parameter_events` and
`/rosout`, multicast discovery is not bridged; re-check the DDS relay and the
route to `192.168.0.0/24`.

### 5. Launch Xvfb, x11vnc, And RViz

This keeps RViz entirely inside the VM and exposes only a VNC desktop to macOS.

```bash
limactl shell autonav-rviz bash -lc '
  pkill -x rviz2 2>/dev/null || true
  pkill -x x11vnc 2>/dev/null || true
  pkill -x Xvfb 2>/dev/null || true
  pkill -x openbox 2>/dev/null || true
  rm -f /tmp/.X3-lock /tmp/.X11-unix/X3

  Xvfb :3 -screen 0 1600x1000x24 +extension GLX +render -noreset \
    >/tmp/xvfb_rviz.log 2>&1 &
  sleep 1
  DISPLAY=:3 openbox >/tmp/openbox_rviz.log 2>&1 &
  sleep 1
  x11vnc -display :3 -localhost -rfbport 5903 -passwd autonav \
    -forever -shared -threads -noxrecord -nowf -nowcr -wait 8 -defer 5 \
    -bg -o /tmp/x11vnc_rviz.log >/tmp/x11vnc_start.log 2>&1

  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export DISPLAY=:3 QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1
  nohup rviz2 -d /Users/cole/code/git/AutoNav_25-26/isaac_ros-dev/config/real_robot_nav_debug.rviz \
    >/tmp/rviz2_real_robot.log 2>&1 &

  sleep 6
  pgrep -af "Xvfb :3|x11vnc|openbox|rviz2"
  tail -80 /tmp/rviz2_real_robot.log
'
```

### 6. Connect With RealVNC Viewer

Use **RealVNC Viewer**, not Apple Screen Sharing. Screen Sharing works for
display updates but can accumulate a large framebuffer backlog, making clicks,
dragging, zooming, and fullscreening feel broken even though sensor data looks
live.

```bash
osascript -e 'quit app "Screen Sharing"' >/dev/null 2>&1 || true
open -a 'VNC Viewer' --args localhost:5903 \
  WarnUnencrypted=0 VerifyId=0 Quality=Low PreferredEncoding=ZRLE2 \
  SendPointerEvents=1 SendKeyEvents=1 Shared=1 Scaling=FitAutoAspect
```

Password: `autonav`

The 2D Goal Pose tool in RViz publishes to `/goal_pose`.

## Recommended RViz Displays

For real robot line/obstacle debugging, use a config that includes:

- fixed frame: `map`
- `/cloud_all_fields_fullframe` as `PointCloud2`, best effort, intensity color
- `/scan_pca_filtered` as `LaserScan`, best effort
- `/scan_pca_filtered_points` as `PointCloud2`, best effort
- `/lidar_line_detection/debug/points` as `PointCloud2`, best effort
- `/lidar_line_costmap`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/plan` and `/local_plan` if available
- TF, robot model, odometry
- 2D Goal Pose tool topic: `/goal_pose`

Avoid a reliable QoS subscription to `/lidar_line_detection/debug/points`; the
publisher is best effort and RViz will warn that the QoS is incompatible.

## Performance Notes

The working viewer path still uses software OpenGL in Xvfb:

```bash
limactl shell autonav-rviz bash -lc 'DISPLAY=:3 glxinfo -B | grep -E "Device|Accelerated|OpenGL renderer"'
```

Expect `llvmpipe` and `Accelerated: no`. That is acceptable for live viewing,
but it makes VNC encoding sensitive to full-screen redraws and high-resolution
framebuffers.

If interaction gets slow:

1. Close Apple Screen Sharing and use RealVNC Viewer.
2. Restart only x11vnc to clear stale clients/backlog:

   ```bash
   limactl shell autonav-rviz bash -lc '
     pkill -x x11vnc 2>/dev/null || true
     x11vnc -display :3 -localhost -rfbport 5903 -passwd autonav \
       -forever -shared -threads -noxrecord -nowf -nowcr -wait 8 -defer 5 \
       -bg -o /tmp/x11vnc_rviz.log >/tmp/x11vnc_start.log 2>&1
   '
   ```

3. Check VNC queues:

   ```bash
   limactl shell autonav-rviz bash -lc 'ss -tinp | grep -A3 5903 || true'
   ```

   Large `Send-Q`, `Recv-Q`, or `notsent` values mean the viewer path is
   backed up.

4. Reduce display load temporarily:
   - hide `/cloud_all_fields_fullframe` while moving the camera
   - lower the Xvfb resolution, for example `1280x800x24`
   - keep the window unscaled when possible

The raw full cloud is usually about `0.5 MB` at `10 Hz`; this is fine for data
transport but expensive for software OpenGL and VNC redraws.

## Troubleshooting

### VM cannot ping the Jetson

Check the Mac USB interface:

```bash
ifconfig en9 | grep 192.168.55.100
ping -c 2 192.168.55.1
```

If the Lima `robotusb` bridge points at the wrong interface, delete and recreate
it:

```bash
limactl network delete --force robotusb
limactl network create robotusb --mode bridged --interface en9
limactl sudoers > /tmp/lima_sudoers
sudo install -o root -g wheel -m 0444 /tmp/lima_sudoers /private/etc/sudoers.d/lima
limactl stop autonav-rviz
limactl start autonav-rviz
```

### VM sees only `/parameter_events` and `/rosout`

Verify direct connectivity and DDS discovery:

```bash
limactl shell autonav-rviz bash -lc '
  ping -c 2 192.168.55.1
  ping -c 2 192.168.0.2
  pgrep -af "^autonav_dds_relay"
  tail -20 /tmp/dds_multicast_relay_rviz.log
'
ssh jetson 'pgrep -af "^autonav_dds_relay"; tail -20 /tmp/dds_multicast_relay_rviz.log'
```

The relay must be running on both the VM and the Jetson host.

### RViz starts but point displays have QoS warnings

Use best effort QoS for LiDAR and line point-cloud displays. The important one
we hit during testing was:

```text
/lidar_line_detection/debug/points
```

### RViz cannot render over SSH/X11 from the Jetson

Do not spend time on this path. XQuartz indirect GL exposes insufficient OpenGL
for RViz/Ogre on this setup, even with `+iglx` and software rendering. The
working approach is RViz inside the Linux VM with VNC to the Mac.

### Apple Screen Sharing looks live but input is unusable

This is the classic failure mode. Sensor updates may look near real-time, while
clicks, dragging, zooming, and fullscreen lag badly. Use RealVNC Viewer and
restart x11vnc with the low-latency options above.

## Cleanup

Stop just the viewer components:

```bash
limactl shell autonav-rviz bash -lc '
  pkill -x rviz2 2>/dev/null || true
  pkill -x x11vnc 2>/dev/null || true
  pkill -x Xvfb 2>/dev/null || true
  pkill -x openbox 2>/dev/null || true
'
ssh jetson 'pkill -f "^autonav_dds_relay" 2>/dev/null || true'
limactl shell autonav-rviz bash -lc 'pkill -f "^autonav_dds_relay" 2>/dev/null || true'
```

Stop the VM:

```bash
limactl stop autonav-rviz
```
