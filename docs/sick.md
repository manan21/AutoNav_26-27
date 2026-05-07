# SICK MultiScan-100 LiDAR

3D LiDAR connected over Ethernet on `eno1`. LiDAR IP `192.168.0.1`, Jetson IP `192.168.0.2`. **The bringup is largely fire-and-forget — almost everything that used to require manual setup is now handled automatically.**

## Bringup

Two equivalent paths:

- **GUI** — click the **Lidar** button in the launch panel.
- **Manual** — `./config/run-lidar.sh` inside the container.

Both paths reconfigure `eno1`, launch the SICK driver, and emit `[GUI_READY] Lidar` after a 5 s pacing delay. The GUI flips the dot green when it sees that sentinel; if it doesn't arrive within 45 s the device is marked failed but kept running so you can read its logs (`hud_node.py:514`).

## Topics

| Topic | Type | Frame | Consumer |
|---|---|---|---|
| `/scan_fullframe` | `sensor_msgs/msg/LaserScan` | `lidar_footprint` | SLAM Toolbox, HUD |
| `/cloud_all_fields_fullframe` | `sensor_msgs/msg/PointCloud2` | `lidar_footprint` | PCA grade detector (`autonav_detection`) |
| `/scan_pca_filtered_points` | `sensor_msgs/msg/PointCloud2` | (transformed) | Nav2 `ObstacleLayer` (local + global costmaps) |

`/scan_pca_filtered_points` is **not** raw LiDAR — it's the grade detector's output after PCA-filtering against ground/ramp slopes. Nav2 sees obstacles, not raw points. SLAM gets the raw 2D scan directly.

The `/scan` vs `/scan_fullframe` historical inconsistency is **resolved** — `slam.yaml`, the active Nav2 params (`nav2_paramsv2.yaml`), and the HUD all reference `/scan_fullframe` consistently. Some dead-code legacy YAMLs (`nav_defaults.yaml`, `nav.yaml`) still mention `/scan` but aren't loaded. `pointcloud_to_laserscan` is also not in the active pipeline — SLAM consumes the SICK driver's native LaserScan output.

## TF

The LiDAR is mounted **upside-down** on the chassis. The URDF (`isaac_ros-dev/src/bringup/description/`) defines a fixed `base_link → lidar_footprint` joint with a π roll, and the SICK driver publishes directly in `lidar_footprint` (no separate `lidar_link`).

## What's automated (don't worry about it)

- **Network config on `eno1`** — done in two redundant places:
  - Container-init: `env/docker/entrypoint_additions/configure-LiDAR.sh` runs once during the Docker first-boot block in `entrypoint.sh`.
  - Per-launch: `./config/run-lidar.sh` flushes and re-applies `192.168.0.2/24` on every Lidar button click, so even a dropped interface gets repaired by toggling the device off/on.
- **Driver respawn** — `sick_multiscan.launch` declares the node `required="true"`; ROS2 restarts it if it dies.
- **UDP timeout handling** — `udp_timeout_ms_initial:=60000` (60 s grace after startup) and `udp_timeout_ms:=10000` (10 s steady-state).
- **Receiver-IP self-check** — `check_udp_receiver_ip:=True` makes the driver validate the UDP path on port 2116 before accepting data.
- **Process cleanup** — the GUI's kill path sends `SIGINT`, then `SIGKILL` after 5 s.

## What still needs you

Code can't fix physical or configuration mismatches:

- **Power / cable** — the driver waits indefinitely for UDP packets that never come. If the LiDAR is unplugged or off, no error is raised; just no data appears.
- **LiDAR IP** — if the unit has been reconfigured off `192.168.0.1`, the driver fails silently. Check the device with the SICK config tool.
- **UDP port conflict** — if something else on the host is bound to port 2115, the driver exits and the GUI flags it red.

## Manual launch (for debugging)

Bypass the run script:
```bash
ros2 launch sick_scan_xd sick_multiscan.launch.py \
    hostname:=192.168.0.1 \
    udp_receiver_ip:=192.168.0.2 \
    publish_frame_id:=lidar_footprint \
    tf_publish_rate:=0
```

The arg is **`udp_receiver_ip`** (the old README incorrectly used `udp_receiver_id` — that silently does nothing). `tf_publish_rate:=0` disables the driver's built-in TF broadcast because the URDF chain already provides `base_link → lidar_footprint`.

If `eno1` isn't configured for some reason and toggling the GUI button doesn't help:
```bash
sudo ip addr flush dev eno1
sudo ip addr add 192.168.0.2/24 dev eno1
sudo ip link set eno1 up
```
