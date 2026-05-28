# Sensors

Single reference for every sensor wired into the AutoNav stack. Hardware, bringup, topics, frames, gotchas — one stop instead of chasing individual doc files.

**Contents**

- [SICK MultiScan-100 LiDAR](#sick-multiscan-100-lidar)
- [ZED 2i Camera](#zed-2i-camera)
- [u-blox ZED-F9P GPS](#u-blox-zed-f9p-gps)
- [Wheel encoders + wheel odometry](#wheel-encoders--wheel-odometry)
- [Power Monitoring PCB](#power-monitoring-pcb)

---

## SICK MultiScan-100 LiDAR

3D LiDAR connected over Ethernet on `eno1`. LiDAR IP `192.168.0.1`, Jetson IP `192.168.0.2`. **The bringup is largely fire-and-forget — almost everything that used to require manual setup is now handled automatically.**

### Bringup

Two equivalent paths:

- **GUI** — click the **Lidar** button in the launch panel.
- **Manual** — `./config/run-lidar.sh` inside the container.

Both paths reconfigure `eno1`, launch the SICK driver, and emit `[GUI_READY] Lidar` 0.5 s after the launch starts (the `sleep 0.5` in `run-lidar.sh`). The GUI flips the dot green when it sees that sentinel; the actual time-to-green is dominated by driver init (UDP handshake, scan grab) and typically lands a few seconds in. If the sentinel doesn't arrive within 45 s the device is marked failed but kept running so you can read its logs (see `_ready_timeouts['Lidar']` in `hud_node.py`).

### Topics

| Topic | Type | Frame | Consumer |
|---|---|---|---|
| `/scan_fullframe` | `sensor_msgs/msg/LaserScan` | `lidar_footprint` | SLAM Toolbox, HUD |
| `/cloud_all_fields_fullframe` | `sensor_msgs/msg/PointCloud2` | `lidar_footprint` | PCA grade detector (`autonav_detection`) |
| `/scan_pca_filtered_points` | `sensor_msgs/msg/PointCloud2` | `lidar_footprint` | `pointcloud_to_laserscan` converters in `slam.launch.py` |
| `/scan_pca_filtered` | `sensor_msgs/msg/LaserScan` | `base_link` | Nav2 local `ObstacleLayer` marking source |
| `/scan_pca_filtered_clear` | `sensor_msgs/msg/LaserScan` | `base_link` | Nav2 local `ObstacleLayer` clearing source |
| `/lidar_line_points` | `autonav_interfaces/msg/LinePoints` | `odom` | Nav2 lidar line layer when **LIDAR LINE DETECT** is running |
| `/lidar_line_costmap` | `nav_msgs/msg/OccupancyGrid` | `odom` | Local lidar-line costmap and global line-memory mirror |
| `/sick_scansegment_xd/imu` | `sensor_msgs/msg/Imu` | `lidar_footprint` | `imu_cov_inflator` → `/sick_scansegment_xd/imu_inflated` (see below) |
| `/sick_scansegment_xd/imu_inflated` | `sensor_msgs/msg/Imu` | `lidar_footprint` (republished) | Local EKF (`ekf_local.yaml` `imu0`) |

`/scan_pca_filtered_points` is **not** raw LiDAR — it is the grade detector's output after PCA-filtering against ground/ramp slopes. The point cloud is collapsed to two LaserScans: a 180° marking scan and a narrower 140° clearing scan. Nav2 sees those obstacle scans, not raw points. SLAM gets the raw 2D scan directly.

The `/scan` vs `/scan_fullframe` historical inconsistency is **resolved** — `slam.yaml`, the active Nav2 params (`nav2_paramsv2.yaml`), and the HUD all reference `/scan_fullframe` consistently for raw LiDAR. Some dead-code legacy YAMLs (`nav_defaults.yaml`, `nav.yaml`) still mention `/scan` but are not loaded. `pointcloud_to_laserscan` is active only for the PCA obstacle path, not for SLAM's raw scan input.

### TF

The LiDAR is mounted **upside-down** on the chassis. The URDF (`isaac_ros-dev/src/bringup/description/`) defines a fixed `base_link → lidar_footprint` joint with a π roll, and the SICK driver publishes directly in `lidar_footprint` (no separate `lidar_link`).

The raw `/sick_scansegment_xd/imu` is republished by the `imu_cov_inflator` package as `/sick_scansegment_xd/imu_inflated` — with inflated covariance so the EKF can fuse it without being overrun by an unrealistically confident IMU prior. `ekf_local.yaml` consumes the inflated topic (`imu0: /sick_scansegment_xd/imu_inflated`). See the [2026-05-11 TROUBLESHOOTING entry](./TROUBLESHOOTING.md#2026-05-11--sick-imu-yaw-inverted-upside-down-lidar-mount) for the historical yaw-inversion diagnosis that motivated the original frame-correction work.

### What's automated (don't worry about it)

- **Network config on `eno1`** — done in two redundant places:
  - Container-init: `env/docker/entrypoint_additions/configure-LiDAR.sh` runs once during the Docker first-boot block in `entrypoint.sh`.
  - Per-launch: `./config/run-lidar.sh` flushes and re-applies `192.168.0.2/24` on every Lidar button click, so even a dropped interface gets repaired by toggling the device off/on.
- **Driver respawn** — `sick_multiscan.launch` declares the node `required="true"`; ROS2 restarts it if it dies.
- **UDP timeout handling** — `udp_timeout_ms_initial:=60000` (60 s grace after startup) and `udp_timeout_ms:=10000` (10 s steady-state).
- **Receiver-IP self-check** — `check_udp_receiver_ip:=True` makes the driver validate the UDP path on port 2116 before accepting data.
- **Process cleanup** — the GUI's kill path sends `SIGINT`, then `SIGKILL` after 5 s.

### What still needs you

Code can't fix physical or configuration mismatches:

- **Power / cable** — the driver waits indefinitely for UDP packets that never come. If the LiDAR is unplugged or off, no error is raised; just no data appears.
- **LiDAR IP** — if the unit has been reconfigured off `192.168.0.1`, the driver fails silently. Check the device with the SICK config tool.
- **UDP port conflict** — if something else on the host is bound to port 2115, the driver exits and the GUI flags it red.

### Manual launch (for debugging)

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

---

## ZED 2i Camera

Stereolabs ZED 2i depth camera connected over USB-C. Provides rectified RGB, registered depth, camera intrinsics, and built-in IMU. The ROS2 wrapper is vendored as a **pinned git submodule** — see [Submodule pin](#zed-submodule-pin) below; mismanaging this pin is the single most common way to break the build.

### Bringup

Two equivalent paths:

- **GUI** — click the **Camera** button in the launch panel.
- **Manual** — `./config/run-zed.sh` inside the container.

Both paths invoke `ros2 launch zed_wrapper zed_camera.launch.py` with our overrides, then `sleep 0.5 && echo "[GUI_READY] Camera"` (the half-second pacing every `run-*.sh` uses). The GUI flips the dot green when it sees that sentinel; the actual time-to-green is dominated by ZED SDK initialization (camera open, intrinsics load, first depth frame). Deadline is 45 s (see `_ready_timeouts['Camera']` in `hud_node.py`) before the device is marked failed but kept running.

### Launch arguments (run-zed.sh)

| Arg | Value | Why |
|---|---|---|
| `camera_model` | `zed2i` | Tells the wrapper which device profile to load |
| `publish_tf` | `false` | URDF + slam_toolbox own all robot TFs; the wrapper would conflict |
| `publish_map_tf` | `false` | Same — slam_toolbox publishes `map → odom` |
| `ros_params_override_path` | `…/install/bringup/share/bringup/config/zed_override.yaml` | Our parameter overlay (see below) |

### Override config (`zed_override.yaml`)

Lives at `isaac_ros-dev/src/bringup/config/zed_override.yaml` and is loaded *after* the wrapper's `common_stereo.yaml` + `zed2i.yaml` defaults, so it wins.

The notable override is the IMU/sensor publish rate:

```yaml
sensors:
  sensors_pub_rate: 380.0
```

The wrapper's default is **100 Hz** (capped). We raise it to **380 Hz** so the EKF gets gyro updates fast enough for tight short-horizon fusion. This was added in commit `8863842a` ("Added yaml overwrite to increase IMU publish rate"). Don't lower it without coordinating with whoever's tuning EKF noise.

### Topics

| Topic | Type | Consumer |
|---|---|---|
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | `autonav_detection` line detector, HUD live view |
| `/zed/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/msg/CameraInfo` | line detector (intrinsics for depth → XYZ projection) |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/msg/Image` | line detector (depth parallax to recover XYZ per pixel) |
| `/zed/zed_node/imu/data` | `sensor_msgs/msg/Imu` | EKF (subject to remap; see `ekf_local.yaml`) |
| `/zed/zed_node/depth/depth_info` | `sensor_msgs/msg/CameraInfo` | depth intrinsics (available; not currently consumed) |

The line detector specifically requires all three RGB-rect/depth/camera-info streams to be time-aligned within `max_rgb_depth_delta_ms: 120` (see `line_detector.yaml`); otherwise it drops the frame.

### TF

The camera link is defined in the URDF (`isaac_ros-dev/src/bringup/description/`) — fixed `base_link → zed_camera_link`, mounted **0.4 m forward** of `base_link` and tilted **~45° downward** (rpy `0 0.785398 0`). Because `publish_tf:=false`, the ZED wrapper does **not** broadcast its own frames — they come from the URDF's `robot_state_publisher`.

### Setup expectations

Everything runs inside the Docker container — you do not need to install the ZED SDK on your laptop. The container ships with the matching SDK and the pinned wrapper builds against it. Your laptop only needs SSH and (optionally) a local ROS2 install for RViz.

### ZED submodule pin

`isaac_ros-dev/src/zed-ros2-wrapper` is a git submodule pinned to a specific Stereolabs release tag. **Do not** track upstream `master` — Stereolabs advances master to whatever SDK they're currently developing against, and a mismatch between the wrapper code and the ZED SDK installed on the Jetson breaks the colcon build.

#### Current pin

| Field | Value |
|---|---|
| Tag | `v5.2.0` |
| SHA | `506e04757fdee442f055ddf280dfd36875732623` |
| Tagged | 2026-02-10 |
| Compatible ZED SDK | 5.1.x and 5.2.x |
| Provides packages | `zed_components`, `zed_wrapper`, `zed_ros2`, `zed_debug` |

The Jetson currently runs ZED SDK **5.1.2**. Tag `v5.2.0` is the latest release that does **not** reference `sl::CustomObjectDetectionProperties::object_tracking_parameters` (that nested struct only exists in SDK 5.2), so it is the newest tag that builds cleanly against 5.1.x while still including the `zed_debug` helper package.

#### How the lock works

`.gitmodules` has only the URL — no branch, no tag:

```ini
[submodule "isaac_ros-dev/src/zed-ros2-wrapper"]
    path = isaac_ros-dev/src/zed-ros2-wrapper
    url = https://github.com/stereolabs/zed-ros2-wrapper.git
```

That's intentional. The submodule is pinned by **SHA only** — the parent repo records the exact commit hash in its tree, not a symbolic reference. Branch tracking would defeat the lock. So the only correct way to fetch the pinned wrapper is:

```bash
git submodule update --init isaac_ros-dev/src/zed-ros2-wrapper
```

This reads the SHA from the parent's git index and checks out exactly that commit (`506e047`).

**NEVER run** `git submodule update --remote` — the `--remote` flag tells git to ignore the recorded SHA and fetch the latest commit on whatever branch is configured upstream. With no branch configured here, it defaults to `master` at Stereolabs, which is on the SDK 5.2 ABI and will break the build against the installed 5.1.x headers.

#### Verifying your pin

Check your local submodule against the parent's recorded SHA:

```bash
git submodule status isaac_ros-dev/src/zed-ros2-wrapper
```

**Clean** (good):
```
 506e04757fdee442f055ddf280dfd36875732623 isaac_ros-dev/src/zed-ros2-wrapper (v5.2.0)
```

**Drifted** (build will break):
```
+bb9d9cddbc0279dae68d77a0ea647ceea12e44c0 isaac_ros-dev/src/zed-ros2-wrapper
```

The leading `+` means the submodule HEAD is **ahead** of the recorded pin; a leading `-` means it's behind or uninitialized. Either state is a build-breaker waiting to happen.

Recovery is simply re-checking-out the pin from inside the submodule:

```bash
cd isaac_ros-dev/src/zed-ros2-wrapper
git fetch --tags origin
git checkout v5.2.0
cd ../../..
git add isaac_ros-dev/src/zed-ros2-wrapper   # only commit if you intended to bump
```

#### When (and how) to bump the pin

Re-pin only when:

1. The ZED SDK on the Jetson is upgraded, **or**
2. A specific bug fix or feature lands upstream that we need.

For an SDK upgrade to 5.2.x, re-pin to `v5.2.2` or newer (those tags target the SDK 5.2 ABI). For an SDK downgrade to 5.0.x, re-pin to `humble-v5.0.0`.

Procedure:

```bash
cd isaac_ros-dev/src/zed-ros2-wrapper
git fetch --tags origin
git checkout <new-tag>           # e.g. v5.2.2
cd ../../..

# Verify the build inside the container before committing:
docker exec -u admin koopa-kingdom bash -lc \
  'cd /autonav/isaac_ros-dev && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select zed_components'

# If clean, commit the new pointer:
git add isaac_ros-dev/src/zed-ros2-wrapper
git commit -m "Pin zed-ros2-wrapper to <new-tag> for SDK <x.y> compatibility"
```

After committing, **update the [Current pin](#current-pin) table above** in the same commit so this doc stays in sync with the actual pin.

#### Symptoms of an unintended bump

If someone runs `git submodule update --remote` or hand-checks-out master inside the submodule and commits the parent, the next `colcon build` will fail with errors like:

```
error: 'struct sl::CustomObjectDetectionProperties' has no member named 'object_tracking_parameters'
```

against the SDK 5.1 headers. Recovery: `git checkout v5.2.0` inside the submodule and re-commit the parent.

This has happened before — see commits `a8f7f708` (the unintended bump), `51cd0917` (the fix), and `fb78812c` (the doc update). Don't be the next one.

### ZED further reading

- ZED ROS2 wrapper docs — <https://www.stereolabs.com/docs/ros2/zed-node>
- ZED node parameter reference — <https://www.stereolabs.com/docs/ros2/020_zed-node>
- ZED SDK install (only needed if you're rebuilding the container image) — <https://www.stereolabs.com/docs/installation/linux>

---

## u-blox ZED-F9P GPS

RTK-capable u-blox ZED-F9P receiver connected to the Jetson over USB (Cypress USB-Serial dual-channel adapter). Streams NMEA at 38400 baud.

### Bringup

Two equivalent paths:

- **GUI** — click the **GPS** button in the launch panel.
- **Manual** — `./config/run-gps.sh` inside the container.

Both spawn **two** nodes back-to-back, then emit `[GUI_READY] GPS` 0.5 s after the launches start (same `sleep 0.5` pacing as the other run scripts):

| Node | Package | Purpose |
|---|---|---|
| `gps_publisher` | `gps_handler` (C++) | Reads NMEA from the serial port, parses GGA/RMC, publishes `/gps_fix` |
| `gps_handler_node` | `gps_waypoint_handler` (Python) | Accepts GPS waypoints, manages the EKF coldstart heading bootstrap |

### Hardware connection

| | |
|---|---|
| **Receiver** | u-blox ZED-F9P |
| **Adapter** | Cypress USB-Serial Dual Channel (USB ID `04b4:0005`) |
| **Jetson serial device** | `/dev/serial/by-id/usb-Cypress_Semiconductor_USB-Serial__Dual_Channel_-if00` |
| **Baud** | 38400 |

The serial path is hardcoded in `isaac_ros-dev/src/gps_handler/src/gps_publisher.cpp` (`SERIAL_PORT` macro). Replacing the GPS unit or adapter requires updating that macro or moving to a parameter.

### Topics

| Topic | Type | QoS | Consumer |
|---|---|---|---|
| `/gps_fix` | `sensor_msgs/msg/NavSatFix` | `SensorDataQoS` (BEST_EFFORT, KEEP_LAST 5) | Global EKF (via `navsat_transform` in `ekf_global.yaml`), HUD GPS panel |

The QoS profile **must** match between publisher and subscriber. A previous mismatch (publisher RELIABLE/depth=50 + subscriber `sensor_data`) caused FastDDS to register the subscription but never route messages — `ros2 topic info` showed zero subscribers while `ros2 node info` listed the sub. The current `SensorDataQoS` on both sides resolves it. Don't change it without re-checking the consumer side.

### Coldstart bias (heading bootstrap)

`run-gps.sh` starts `gps_handler_node` with `coldstart_bias_enabled:=true` and `coldstart_theta_seed_variance_deg:=45.0`.

On the first GPS goal accepted by the node, the EKF's `θ_offset` is snapped so that the normal `world→odom` projection of the goal lands directly in front of `base_link`. The robot then drives toward the real GPS waypoint at the real GPS distance, accumulates the displacement that `bootstrap_theta` needs to refit θ from data, and the seed is overwritten by the closed-form fit as soon as `baseline > BOOTSTRAP_MIN_BASELINE_M`. The 45° seed variance is intentionally loose so the EKF's first real heading update dominates immediately. One-shot per node — restart re-arms it.

### What still needs you

- **Antenna sky view** — F9P needs a reasonable view of the sky. Tuning indoors will silently degrade the fix to "no fix" with no fault message; check `/gps_fix.status.status` in the HUD's GPS panel.
- **USB port stability** — Cypress USB-Serial sometimes re-enumerates on the wrong tty after a hot-plug. The `by-id` path above is stable, so if it stops resolving the device is physically disconnected.
- **RTK / DGPS corrections** — currently we don't feed corrections; the fix is single-receiver standalone. If a base station is added, that's a configuration change in the receiver (via u-center), not in this code.

---

## Wheel encoders + wheel odometry

The Roboteq motor controller provides the encoder counts; a Jetson-side node integrates them into a wheel odometry message that feeds the local EKF.

### Hardware connection

| | |
|---|---|
| **Motor controller** | Roboteq FBLG2360T (USB ID `20d2:5740`) |
| **Encoders** | Built into the gear motors, wired into the Roboteq's encoder inputs |
| **Comm** | USB-serial to the Jetson; the `control` node opens the device and queries encoder counts at the loop rate |

### Bringup

Both nodes come up via the **Pre-SLAM** GUI button (or `./config/run-pre-slam.sh` manually). They start together because the wheel-odometry node has no useful input until the `control` node is publishing `/encoders`.

| Node | Package | Source | Purpose |
|---|---|---|---|
| `control_node` | `control` | `control.cpp` | Talks to the Roboteq, publishes `/encoders`, owns the joystick / cmd_vel path, manages Phase D grade compensation |
| `wheelodom_publisher` | `odom_handler` | `wheel_odom_pub.cpp` | Subscribes to `/encoders`, integrates wheel kinematics, publishes `/odom` + the `odom → base_link` TF |

### Topics

| Topic | Type | Frame | Publisher | Consumer |
|---|---|---|---|---|
| `/encoders` | `autonav_interfaces/msg/Encoders` | — | `control_node` | `wheelodom_publisher`, automated testing |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_link` | `wheelodom_publisher` | Local EKF (`ekf_local.yaml`), HUD Encoders panel |

The `wheelodom_publisher` also broadcasts the `odom → base_link` TF; the local EKF *replaces* this TF with its own filtered version once it's fused enough samples. Both publishing is intentional — the bare wheel TF is what's on screen before the EKF locks in.

### Wheel parameters (`wheel_odom_pub.cpp` constants)

| Constant | Value | Notes |
|---|---|---|
| `wheel_base_` | `0.6858 m` | Distance between left and right contact patches |
| `wheel_radius_` | `0.12946 m` | Effective wheel radius (measured, not nominal) |
| `ticks_per_revolution_` | `81923` | Roboteq's encoder counts per wheel revolution (post-gearbox) |
| `left_encoder_scale_` | `1 / 1.016335` | **🔴 OSCILLATION-SENSITIVE.** Corrects the left encoder's ~1.6% oversample. Without it, wheel-derived yaw rate is biased, `odom → base_link` drifts in yaw, and slam_toolbox has to scan-match-correct against a moving target → map-snap and stall. **Do not remove or rebalance to 1.0** without rerunning the encoder asymmetry self-diagnostic in `wheel_odom_pub.cpp` and confirming the EMA ratio is ≈ 1.0. |

These are build constants — change them in source and `colcon build --packages-select odom_handler`. The left scale is the load-bearing one; see commit `70e0dcb5` and the `🔴 OSCILLATION-SENSITIVE` block in the source for the full rationale.

### What still needs you

- **Motor controller off** — if the Roboteq is unpowered or the e-stop is engaged, `control_node` connects but encoder counts stay at 0. `/odom` keeps publishing 0 velocity, the EKF stays put, no error surfaces.
- **Encoder rewiring** — if a wheel encoder cable comes loose, that wheel's count freezes. The asymmetry diagnostic flags this once the EMA ratio drifts noticeably from 1.0; manual visual check during driving is the fastest catch.
- **Wheel diameter drift** — tire wear changes `wheel_radius_`. Re-measure annually or after a tire change.

---

## Power Monitoring PCB

> ⚠️ **Currently offline.** I²C is broken on the Jetson — a recent
> attempt to fix it resulted in the Jetson failing to boot, so the
> PCB is physically disconnected pending a Jetson recovery. The
> driver code below still describes the *design*, but no `/electrical/*`
> topics are publishing right now. Don't expect live readings or wire
> anything new to depend on the PCB until I²C is restored.

A custom Texas-Instruments-based PCB that sits **in series between the battery and the rest of the robot**, measures voltage / current / power across a shunt, and reports State of Charge to the Jetson over I²C. Designed and maintained as a separate project (link at the bottom); this section only covers how it hooks into AutoNav.

### How it physically connects

The board has four screw terminals split into two pairs:

| Terminal pair | What it connects to |
|---|---|
| **POWER + / −** | The **battery** pack (currently a Renogy RBT2425LFP LiFePO₄, ~25.6 V nominal) |
| **LOAD + / −** | The **rest of the robot** — motors, Jetson power input, sensors, everything downstream |

So the topology is:

```
[ Battery ]
     │
     ▼
POWER+   POWER−
   ╔════════════╗
   ║  PCB       ║   ← shunt + INA226 measure here
   ║  (INA226 + ║      between POWER side and LOAD side
   ║   BQ34Z100)║
   ╚════════════╝
LOAD+    LOAD−
     │
     ▼
[ Rest of robot ]
```

Every amp the robot draws flows through the PCB's shunt resistor, so current can be inferred from the tiny voltage drop across it. (See the *Power Monitoring PCB (brief synopsis)* section in the [main README](./HUMAN-WRITTEN-README.md#power-monitoring-pcb-brief-synopsis) for the math.)

### How it talks to the Jetson

Three wires, plus you can ignore power for the comm bus because the PCB powers itself off the battery side:

| Wire | Purpose |
|---|---|
| **SCL** | I²C clock |
| **SDA** | I²C data |
| **GND** | Common ground (shared with the Jetson) |

That's it — it's I²C. No USB, no UART, no Ethernet on the comm side.

| | |
|---|---|
| **Jetson I²C bus** | `/dev/i2c-1` (Jetson 40-pin header pins 27 = SDA, 28 = SCL) |
| **INA226 address** | `0x40` |
| **BQ34Z100-R2 address** | (separate; SOC gauge — see PCB repo for details) |

### How it shows up in ROS2

The Jetson-side software lives in the **`autonav_electrical_publisher`** package.

| | |
|---|---|
| **Bringup (GUI)** | Click the **Power PCB** button in the launch panel |
| **Bringup (manual)** | `./config/run-electrical.sh` |
| **Launch file** | `ros2 launch autonav_electrical_publisher electrical_publisher.launch.py` |
| **Node** | `electrical_publisher_node` |
| **Publishes** | `/electrical/voltage` (`std_msgs/msg/Float32`), `/electrical/current`, `/electrical/power` |

Per the package's own gotcha: the INA226 needs a **calibration register write at startup** (`0x05 = 0x0800` for the 10 mΩ shunt). The node retries in a timer loop on I²C init failure — don't trust readings until the log says "calibrated and ready."

### What the PCB does on its own (no Jetson required)

Even with the Jetson off, the PCB drives an LED bank showing approximate SOC (100% / 80% / 60% / 40% / 20% / critical). Useful for a quick "should I plug it in?" check while the robot is parked.

The on-board BQ34Z100-R2 fuel gauge integrates current vs. time and re-anchors against an empirical discharge curve when the battery is at rest — that's the same drift-correction logic described in the [Drift section](./HUMAN-WRITTEN-README.md#drift) of the main README.

### House rule — protective enclosure

After the **R2 board failure on the Bowser → Shogi transfer** (metal shavings from chassis work landed on the bare board, induced CMOS latch-up on the INA226's SCL pin, and cascaded into a dead DC-DC buck), the project mandates that **all future PCBs ship inside a protective enclosure before being mounted on a robot**. Bare boards near a metal chassis is a "when, not if" failure mode.

Concretely: if you see the PCB exposed on a robot, escalate before the next test session.

### Relevant ROS2 package

[`autonav_electrical_publisher`](../isaac_ros-dev/src/autonav_electrical_publisher/) — see the [PACKAGES.md entry](./PACKAGES.md#autonav_electrical_publisher) for build/topic specifics.

### Reference repo

The full PCB design (KiCad project, schematic, BOM/fab outputs, programming scripts, board bring-up procedure, datasheets) lives in a separate repository:

**<https://github.com/nfikes/AutoNav-Charge_Indicator-KiCad_Pcb>**

Read that repo's README for the schematic, the board bring-up procedure (10 V power-on test, voltage-rail verification, I²C comm tests, BQ34Z100 chemistry programming), and the full failure-mode write-ups.
