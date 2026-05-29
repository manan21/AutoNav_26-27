# PACKAGES

A briefing on each ROS2 package under `isaac_ros-dev/src/`. Bigger packages are split into multiple boxes so each section stays scannable.

> Heads up: a few entries in the source tree aren't load-bearing. `autonav_supervisor/` is an empty stub (no `package.xml`). The old Gazebo Classic `sim/` and `autonav_sim/` packages are retired with `COLCON_IGNORE`; the active Gazebo target is `igvc_competition_sim` on ROS Humble + Gazebo Fortress. `pointcloud_to_laserscan/` is a vendored upstream package documented for reference only. See [`docs/LAUNCH_STACK.md`](./LAUNCH_STACK.md) for what actually runs.

## Table of contents

- [autonav_automated_testing](#autonav_automated_testing)
- [autonav_detection](#autonav_detection)
  - [`line_detector` executable](#line_detector-executable)
  - [`grade_detector` executable](#grade_detector-executable)
  - [`lidar_line_detector` executable](#lidar_line_detector-executable)
- [autonav_electrical_publisher](#autonav_electrical_publisher)
- [autonav_interfaces](#autonav_interfaces)
- [autonav_sim](#autonav_sim)
- [autonav-gui-hud](#autonav-gui-hud)
  - [What it is and where it runs](#what-it-is-and-where-it-runs)
  - [Install and launch](#install-and-launch)
  - [Launch panel + readiness handshake](#launch-panel--readiness-handshake)
  - [Docker exec wrapping](#docker-exec-wrapping)
- [bringup](#bringup)
  - [Robot description (URDF + meshes)](#robot-description-urdf--meshes)
  - [Config and launch composition](#config-and-launch-composition)
- [control](#control)
- [custom_behavior_tree_plugins](#custom_behavior_tree_plugins)
- [gps_handler](#gps_handler)
- [gps_waypoint_handler](#gps_waypoint_handler)
- [igvc_competition_sim](#igvc_competition_sim)
- [imu_cov_inflator](#imu_cov_inflator)
- [line_layer](#line_layer)
- [local_mirror_layer](#local_mirror_layer)
- [map_padder](#map_padder)
- [odom_handler](#odom_handler)
- [pointcloud_to_laserscan](#pointcloud_to_laserscan)
- [sick_scan_xd](#sick_scan_xd)
- [sim](#sim)
- [slam](#slam)
  - [Overview](#overview)
  - [Launch files and configs](#launch-files-and-configs)
  - [Behavior trees and the GPS fusion path](#behavior-trees-and-the-gps-fusion-path)
- [zed_components](#zed_components)
- [zed_debug](#zed_debug)
- [zed_ros2](#zed_ros2)
- [zed_wrapper](#zed_wrapper)

---

## autonav_automated_testing

ROS2 testing harness with an RQT GUI for running, recording, and replaying automated robot tests. Orchestrates test runs, collects timestamped CSVs from GPS, IMU, encoders, odometry, and others, and records camera + LiDAR video during the run.

| | |
|---|---|
| **Provides** | RQT plugin `autonav_automated_testing_plugin`, executable `data_publisher`, launches like `t000_DAQ_MODE.launch.py`, `t000_AUTO_DAQ_MODE.launch.py`, `t002_Line_Comp.launch.py` |
| **Publishes** | `/data/dump` (`std_msgs/String` — CSV-formatted sensor data), `/estop` (`std_msgs/String`) |
| **Subscribes** | `/data/toggle_collect` (`std_msgs/Bool`), `/estop`, plus dynamic per-test sensor topics |
| **Configs** | `testing_data_collection_setter.yaml` (per-test-id topic lists), `calibration_constants.yaml` |
| **Build** | `ament_cmake` |

> **Heads up:** Logs go to `/autonav/logs/{test_id}_{timestamp}/` and are loaded back via the "Load Log File" button. The e-stop has two paths — the node listens to `/estop` and the GUI dialog watches for the literal string `STOP`.

---

## autonav_detection

Three perception executables share one ament_cmake package: a CUDA-accelerated camera white-line detector, a pure-C++/Eigen LiDAR PCA grade detector, and a SICK RSSI/reflector lidar-line detector. Built with C++17 + CUDA (Ampere SM 87 for Jetson Orin Nano).

**Top-level files:**

| | |
|---|---|
| **Launch** | `launch/detection.launch.py` (toggle each detector with `enable_line` / `enable_grade` / `enable_lidar_line`) |
| **Configs** | `config/line_detector.yaml`, `config/grade_detector.yaml`, `config/lidar_line_detector.yaml` |
| **Build** | `ament_cmake` with CUDA enabled; `grade_detector` is force-built `-O3 -DNDEBUG -Wno-class-memaccess` (per-cell PCA is ~50–100× slower without `-O3`) |

### `line_detector` executable

CUDA detector for white lines using the CERIAS algorithm (local brightness mean + variance kernel).

| | |
|---|---|
| **Subscribes** | `/zed/zed_node/rgb/color/rect/image`, `/zed/zed_node/depth/depth_registered`, `/zed/zed_node/rgb/color/rect/camera_info` |
| **Publishes** | `/line_points` (`autonav_interfaces/msg/LinePoints`), `/lines_pointcloud` (`PointCloud2`) |
| **Sources** | `src/line/node.cpp`, `detection.cpp`, `cuda.cu` |
| **Tunables** | `brightness_threshold`, `half_window_size`, `sigma_threshold`, `mew_threshold` (all in YAML) |

> **Heads up:** Requires RGB and depth time-aligned within `max_rgb_depth_delta_ms: 120`; otherwise it drops the frame. The target TF frame must exist for depth → map reprojection.

### `grade_detector` executable

LiDAR PCA pipeline that classifies traversable ground vs. ramps/slopes and publishes obstacle points for Nav2.

| | |
|---|---|
| **Subscribes** | `/cloud_all_fields_fullframe` (`PointCloud2`) from `sick_scan_xd` |
| **Publishes** | `/scan_pca_filtered_points` (`PointCloud2`, xyz only) → `pointcloud_to_laserscan` converters → Nav2 `ObstacleLayer` |
| **Debug topics** | `/terrain/grade_map` (`OccupancyGrid`), `/pca/surface_normal` (`Vector3Stamped`) |
| **Sources** | `src/grade/pca_node.cpp`, `pca_pipeline.cpp` |
| **Tunables** | 16.7° traversable cap, 1.5° noise margin, 89° PCA validity cap, 0.3 m DBSCAN eps, 0.1 m grid cells over ±8 m |

> **Heads up:** Pipeline must complete in <60 ms per scan (project rule). Slope math runs in **sensor frame** — do not introduce IMU/world-up. Front-arc-only mode drops 50% of the cloud and is on by default; turn off only for debugging.

### `lidar_line_detector` executable

SICK MultiScan retroreflective tape detector. It uses reflector/RSSI returns, gates them to the floor in `base_link`, clusters tape-like marks, optionally completes sparse ground-only clusters into short local segments, and publishes line points for the lidar line costmap layer.

| | |
|---|---|
| **Subscribes** | `/cloud_all_fields_fullframe` (`PointCloud2`) from `sick_scan_xd` |
| **Publishes** | `/lidar_line_points` (`autonav_interfaces/msg/LinePoints`), `/lidar_line_detection/debug/points`, `/lidar_line_detection/diagnostics` |
| **Sources** | `src/lidar_line/node.cpp` |
| **Tunables** | Ground gate, reflector/intensity thresholds, clustering, voxel output, and segment completion in `config/lidar_line_detector.yaml` |

> **Heads up:** Segment completion runs only after the base-link ground-height gate and only inside accepted clusters. It is intended to fill sparse floor tape, not bridge separate obstacles or raised cone reflectors.

---

## autonav_electrical_publisher

> ⚠️ **Currently non-functional.** I²C is broken on the Jetson — a recent fix attempt left the Jetson unbootable, and the Power PCB is physically disconnected pending recovery. The driver below still describes the design, but **no `/electrical/*` topics are publishing right now**. See the [TROUBLESHOOTING entry](./TROUBLESHOOTING.md#power-pcb-silent--no-electrical-readings) and [`docs/SENSORS.md`](./SENSORS.md#power-monitoring-pcb).

Talks to the on-board Power Monitor PCB (INA226 over I²C) and publishes battery voltage, current, and power.

| | |
|---|---|
| **Provides** | Node `electrical_publisher_node` (executable `electrical_publisher`) + `electrical_publisher.launch.py` |
| **Publishes** | `/electrical/voltage` (V, `Float32`), `/electrical/current` (A), `/electrical/power` (W) |
| **Hardware** | I²C bus `/dev/i2c-1` (Jetson pins 27/28), slave address `0x40` |
| **Build** | `ament_cmake` |

> **Heads up:** The chip needs a calibration register write at startup (`0x05 = 0x0800` for the 10 mΩ shunt). The node retries in a timer loop on I²C init failure — don't trust readings until you see "calibrated and ready" in the logs.

---

## autonav_interfaces

Custom ROS2 message and service definitions used across the project. Pure interface package (member of `rosidl_interface_packages`); no executables.

| Type | Name | Purpose |
|---|---|---|
| msg | `Encoders.msg` | Left/right wheel RPM and tick counts |
| msg | `GpsData.msg` | GNSS lat/lon/alt — **vestigial**, no longer published; live GPS is `sensor_msgs/NavSatFix` on `/gps_fix` |
| msg | `LinePoints.msg` | Timestamped 3D line waypoint array |
| srv | `AnvLines.srv` | Request line waypoints |
| srv | `ConfigureControl.srv` | Toggle Arduino / motor / GPS / e-stop subsystems |
| srv | `GpsToLocal.srv` | Convert lat/lon to a `PoseStamped` in the robot's local frame (served by `gps_handler_node`) |
| srv | `LocalToGps.srv` | Inverse of the above — local `PoseStamped` → lat/lon |
| action | `NavigateToWaypoint.action` | Unified GPS-or-local navigation goal (`goal_type` discriminator + `success_radius_m`). Single endpoint for both `send_GPS_waypoint.sh` and local-pose missions. See file header for the full status enum and feedback fields |

| | |
|---|---|
| **Build** | `ament_cmake` + `rosidl_default_generators` |

> **Heads up:** These exist because the standard ROS message set doesn't cover encoder ticks, our line-detection format, or the unified GPS/local action shape. Consumed by `control` (Encoders), `gps_waypoint_handler` (NavigateToWaypoint, GpsToLocal, LocalToGps), and `line_layer` / `autonav_detection` (LinePoints).

---

## autonav_sim

**Retired Gazebo Classic assets.** This package has `COLCON_IGNORE`; use [`igvc_competition_sim`](#igvc_competition_sim) for the active ROS Humble + Gazebo Fortress simulation.

| | |
|---|---|
| **Robot descriptions** | `description/custom_robot/`, `description/tester_robot/` |
| **Worlds** | `worlds/autonav_igvc_course.world`, `worlds/empty.world` |
| **Models** | IGVC road sections + a resized `construction_barrel` |
| **Build** | `ament_cmake` |

> **Heads up:** Kept around for reference / rollback, not in active use. It is intentionally ignored by colcon.

---

## igvc_competition_sim

Active Gazebo Fortress IGVC competition simulation. It runs the Humble robot stack against a compact, deterministic IGVC-style course with a rendered ZED-style RGB-D camera, plain white tape, PCA obstacle source, GPS waypoint, costmap, BT, planner, controller, and scoring tests.

| | |
|---|---|
| **Simulator** | Gazebo Fortress through `gz sim` / `ign gazebo` with `ros_gz_bridge` |
| **Launch** | `launch/igvc_competition.launch.py` |
| **Run script** | `Run_IGVC_COMPETITION_FORTRESS_TEST.command` |
| **Course source** | `config/igvc_competition_compact.yaml` |
| **World** | `worlds/igvc_competition_compact.sdf`, generated by `generate_igvc_world` |
| **Executables** | `igvc_sensor_harness`, `igvc_camera_bridge`, `igvc_course_monitor`, `igvc_mission_runner`, `generate_igvc_world` |
| **Build** | `ament_python` |

> **Heads up:** Gazebo provides the world/physics target and RGB-D camera. `igvc_camera_bridge` relays that camera into the real ZED topics for the CUDA line detector, while `igvc_sensor_harness` publishes the SICK-style point cloud, PCA obstacle source, GPS fix, odom/TF, map, and joint states. Use `line_detection_mode:=ground_truth` when isolating Nav2 from camera perception.

---

## autonav-gui-hud

The native Jetson HUD that drives the entire stack. Big package — split into four boxes below.

### What it is and where it runs

A 1920×720 PyQt5 control panel. **Runs natively on the Jetson host, NOT in the container.** It talks to the containerized ROS2 system two ways: DDS for live topic subscriptions/publications, and `docker exec` for launching processes inside the container. Provides device control, live telemetry, playback analysis, and stack status.

### Install and launch

| | |
|---|---|
| **One-time install** | `sudo ./install.sh` (PyQt5, matplotlib, opencv-headless, numpy, PIL) |
| **Launch** | `./isaac_ros-dev/config/run-gui.sh` (wrapper) or `./run_gui.sh` (direct) |
| **Build** | `ament_python` (entry point: `hud_node = autonav_gui_hud.hud_node:main`) |

The launchers source ROS2, set `ROS_DOMAIN_ID`, point Qt5 at the system plugin path, and pick a `DISPLAY` (`:0` by default; X11 forwarding works for remote use).

### Launch panel + readiness handshake

Ten buttons in queue order on the production branch: **Pre-SLAM, Camera, Lidar, GPS, PCA DETECT, CAMERA LINE DETECT, LIDAR LINE DETECT (opt-in — excluded from Run All), SLAM, NAV2, Power PCB**. Each toggles a script (e.g. `./config/run-zed.sh`); the queue advances when a script prints `[GUI_READY] <Label>` on stdout. Default per-device timeout is 60 s; longer ones are listed in `_ready_timeouts` (Camera/Lidar 45 s, SLAM 120 s, GPS 300 s). See [`docs/LAUNCH_STACK.md`](./LAUNCH_STACK.md) for the full table with dependency reasoning.

> **Heads up:** Status dots — gray = off, yellow = starting, green = ready. After ~5 s the GUI starts polling stdout for the sentinel; if it never arrives within the device's deadline, the dot turns red but the process is left running so you can read the logs.

### Docker exec wrapping

Every container-side command goes through `_wrap_container_cmd` (`hud_node.py:2420`):

```
docker exec -u admin --workdir /autonav/isaac_ros-dev koopa-kingdom \
    /bin/bash -lc 'echo $$ > /tmp/gui_pid_<label> && \
                   source /opt/ros/humble/setup.bash && \
                   source install/setup.bash && \
                   exec <cmd>'
```

The PID file lets the kill path send `SIGINT` and then `SIGKILL` after 5 s on tree-of-children processes.

> **Heads up:** Container-dependent buttons require **Connect to Container** to have been clicked first. Without that, the launch buttons can't fire — the GUI has no way to `docker exec`.

---

## bringup

System orchestration package — owns the URDF and composes everyone else's launch files. Two boxes.

### Robot description (URDF + meshes)

Defines the TF tree foundation; everyone else (`sensors`, `control`, `slam`, Nav2) builds on top of it.

| | |
|---|---|
| **Primary URDF** | `description/bowser.urdf.xacro` (Xacro, with mesh-based links and sensor frames: ZED 0.4 m forward + 45° pitch, SICK rear, inverted) |
| **Alt URDF** | `description/shogi.urdf` (the model currently set as default in `core_bringup.launch.py`) |
| **Meshes** | `description/meshes/*.STL` (8 files: base, wheels, caster, camera, GPS, lidar) |

### Config and launch composition

| | |
|---|---|
| **Config** | `config/zed_override.yaml` — raises `sensors_pub_rate` to 380 Hz so the EKF gets fast IMU updates |
| **Launch — `core_bringup.launch.py`** | `robot_state_publisher` + `joint_state_publisher`, broadcasts the TF tree |
| **Launch — `pre_slam.launch.py`** | joy + core + control + wheel odom (entry point for `run-pre-slam.sh`) |
| **Launch — `sensors.launch.py`** | Composes ZED + SICK with hardware IPs |
| **Launch — `bringup.launch.py`** | Full stack: core + SLAM + sensors + control + NAV2 |
| **Launch — `demo_day.launch.py`** | Competition profile with curated args + line/grade detection wired in |
| **Build** | `ament_cmake`; exec_depends include `joint_state_publisher`, `robot_state_publisher`, `xacro`, `launch`, `launch_ros`, `ament_index_python` |

> **Heads up:** No nodes of its own — purely the assembly layer.

---

## control

Bridges Xbox joystick + autonomy commands to the motor controller. Manages mode switching (manual ↔ autonomous), serial to a Roboteq, and a small Arduino link.

| | |
|---|---|
| **Provides** | Node `control` + `control_dev.launch.py` |
| **Subscribes** | `/joy`, `/cmd_vel` (from Nav2), `/estop` |
| **Publishes** | `/encoders` (`autonav_interfaces/msg/Encoders` at 30 Hz), `/autonomous_mode` (`Bool`), `/motor_speed` (`String`, for DAQ logging) |
| **Service** | `ConfigureControl` from `autonav_interfaces` (toggle Arduino / motors at startup) |
| **Hardware** | Roboteq motor controller over USB serial @ 115200 (`/dev/serial/by-id/usb-RoboteQ...`); Arduino @ 9600 baud for mode messages |
| **Build** | `ament_cmake` |

> **Heads up:** **X** on the Xbox controller (rising edge) flips `autonomousMode` — the motor source switches from joystick sticks to `/cmd_vel` Twist. **B** triggers the e-stop via `/estop`, which calls `motors.shutdown()` immediately.

---

## custom_behavior_tree_plugins

Custom Nav2 behavior and BT plugins that extend planning validation, forward/reverse dispatch, and recovery behavior.

| Plugin | What it does |
|---|---|
| `gradient_escape` | Samples the local costmap in N directions (default 16), drives at 0.1 m/s toward the lowest-cost cell until cost drops below the configured threshold. Returns SUCCESS on escape, FAILED after timeout. Exposed as a Nav2 `Behavior` through the `DriveOnHeading` action type. |
| `breadcrumb_reverse` | Nav2 behavior that reverses to the most recent breadcrumb from `/breadcrumb_tail` when the global path leads behind the robot. Checks the global costmap before committing to a rearward move. |
| `goal_bender` | Bends a behind-the-robot goal or U-turn path toward a forward intermediate point so the robot arcs back instead of pivoting in place. |
| `is_forward_blocked` | BT condition used by `bt_nav.xml` to decide whether the current path leads forward enough for `FollowPath`, or whether the robot should spend a breadcrumb first. |
| `path_footprint_safe` | BT condition that checks each path pose's padded rectangular `nav_center` footprint against `/global_costmap/costmap_raw` and rejects paths that overlap lethal global cells before MPPI can execute them. |
| `path_significantly_changed` | Dormant decorator retained for experiments; the active `bt_nav.xml` no longer wraps `FollowPath` with it because MPPI should receive each fresh safe path directly. |

| | |
|---|---|
| **Wired in** | `slam/behavior_trees/bt_nav.xml` and `nav2_params_camera.yaml` plugin lists |
| **Build** | `ament_cmake`; produces Nav2 behavior libraries and BT factory plugins including `gradient_escape_core`, `breadcrumb_reverse_core`, `autonav_goal_bender_bt_node`, `autonav_is_forward_blocked_bt_node`, `autonav_path_footprint_safe_bt_node`, and `autonav_path_significantly_changed_bt_node` |

> **Heads up:** The active navigation stack relies on `PathFootprintSafe` as a hard gate: Smac Lattice should already produce footprint-aware paths, but this BT check prevents a bad path from reaching MPPI if the raw global costmap says the body would overlap tape or an obstacle.

---

## gps_handler

C++ driver for the u-blox ZED-F9P GPS. Talks NMEA (GGA + RMC) over serial, publishes `NavSatFix`. Mission/waypoint logic is **not** here — see `gps_waypoint_handler` for the action server that consumes `/gps_fix`.

| | |
|---|---|
| **Provides** | Executable `gps_publisher` (`src/gps_publisher.cpp`) |
| **Hardware** | `/dev/serial/by-id/usb-Cypress_Semiconductor_USB-Serial__Dual_Channel_-if00` @ 38400 baud |
| **Publishes** | `/gps_fix` (`sensor_msgs/msg/NavSatFix`) at 10 Hz |
| **Vendored libs** | `serialib.{cpp,hpp}` (also duplicated in `odom_handler` — not shared) |
| **Build** | `ament_cmake` |

> **Heads up:** Driver only. Reconnects automatically on parse errors / serial hangups (see TROUBLESHOOTING.md 2026-04-28 `stoi` crash entry).

---

## gps_waypoint_handler

Self-correcting GPS waypoint **action server** with a magnetometer-less heading EKF. Replaces the older "read a file, call `BasicNavigator.followWaypoints`" mission layer — goals now arrive as `/navigate_to_waypoint` actions from anywhere in the stack (GUI, `send_GPS_waypoint.sh`, mission BTs).

| Console script | Role |
|---|---|
| `gps_handler_node` | The action server. Owns the heading EKF, candidate-goal smoother, and the `/goal_pose` ↔ `/goal_update` republisher |
| `get_gps_positioning` | Captures current `/gps_fix` to `cur_gps_positon.txt` ("record this waypoint") |

Python modules:

| Module | What it does |
|---|---|
| `gps_handler_node.py` | Main node (~2.6 k lines). Action server, two callback groups (action vs estimator), threaded EKF heartbeat, candidate-goal pipeline (EWMA → 1/r envelope → moving-away trip-wire → force-resync) |
| `gps_ekf.py` | Magnetometer-less heading EKF: closed-form θ fit over rolling motion samples + Kalman update. Anchors the imaginary "fake-north" ENU frame to the real GPS frame |
| `gps_conversions.py` | Lat/lon ↔ local Cartesian (used by the `GpsToLocal` / `LocalToGps` services) |
| `get_gps_positioning.py` | One-shot helper for recording current GPS position to a file |

| | |
|---|---|
| **Action** | `/navigate_to_waypoint` (`autonav_interfaces/action/NavigateToWaypoint`) — unified GPS-or-local goal with `goal_type` discriminator + per-goal `success_radius_m` |
| **Services** | `/gps_waypoint/gps_to_local`, `/gps_waypoint/local_to_gps` (`autonav_interfaces/srv`) |
| **Subscribes** | `/gps_fix` (sensor QoS), `/odometry/filtered` (EKF heartbeat at ~30 Hz), `/gps_waypoint/next_hint` (look-ahead for chained legs) |
| **Publishes** | `/goal_pose` at 1 Hz on leg start (NAV2 trigger); `/goal_update` at 1 Hz in-mission (consumed by the `GoalUpdater` in `bt_nav.xml` — rewrites the live goal without canceling FollowPath); diagnostics on `/gps_waypoint/heading_offset`, `…/heading_offset_std_deg`, `…/debug`, `…/candidate_marker`, `…/health` |
| **Data** | `cur_gps_positon.txt` (only used by the recorder; the action server takes goals over the action interface, not from a file) |
| **Depends on** | `rclpy`, `autonav_interfaces`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`, `visualization_msgs` |
| **Build** | `ament_python` |

> **Heads up:** `run-gps.sh` starts the C++ `gps_publisher` *and* `gps_handler_node` together, passing `coldstart_bias_enabled:=true` so the first GPS goal snaps θ_offset to land the waypoint in front of `base_link` — the EKF then overwrites that seed once the robot accumulates motion baseline. See `docs/HUMAN-WRITTEN-README.md` "Self-Correcting GPS" for the algorithm synopsis.

---

## imu_cov_inflator

Python (`ament_python`) node that republishes an IMU topic with inflated covariance so `robot_localization`'s EKF can fuse it without being overrun by an unrealistically confident zero-covariance source. Used for the SICK MultiScan's onboard IMU, which is mounted under a π-roll relative to `base_link`.

| | |
|---|---|
| **Source** | `imu_cov_inflator/imu_inflator_node.py` |
| **Subscribes** | `/sick_scansegment_xd/imu` (raw, in `lidar_footprint` frame) |
| **Publishes** | `/sick_scansegment_xd/imu_inflated` (republished with bumped covariance) |
| **Build** | `ament_python` |
| **Consumed by** | Local EKF — `ekf_local.yaml` sets `imu0: /sick_scansegment_xd/imu_inflated` |

See the [2026-05-11 TROUBLESHOOTING entry](./TROUBLESHOOTING.md#2026-05-11--sick-imu-yaw--covariance-handling-historical-mechanism-since-superseded) for the historical context — this package supersedes an earlier `imu_frame_transformer.py` that did a hard frame rotation instead of trust-weighting.

---

## line_layer

Nav2 costmap plugin (not a node) that paints detected line points into the costmap as lethal obstacle seeds plus a configurable soft halo.

| | |
|---|---|
| **Plugin manifest** | `line_layer.xml` (exports `line_layer::LineLayer`, base `nav2_costmap_2d::Layer`) |
| **Sources** | `src/line_layer.cpp`, headers `include/line_layer/{line_layer.hpp, line_buffer.hpp}` |
| **Library output** | `line_layer_core` |
| **Subscribes** | Configured `line_topic` (`/line_points` for camera lines or `/lidar_line_points` for LiDAR tape) |
| **Publishes** | Optional line-only costmap topic such as `/line_costmap` or `/lidar_line_costmap` |
| **Loaded by** | Nav2 costmap config. `nav2_params_camera.yaml` enables `line_layer`; `nav2_params_lidar.yaml` enables `lidar_line_layer`. |
| **Build** | `ament_cmake` |

> **Heads up:** Plugin, not node. Exact line cells are lethal; the line layer's own halo is intentionally narrower than stock obstacle inflation. The active camera profile uses view-gated camera-line memory; the opt-in lidar profile keeps temporary manual-clear-only lidar-line memory for retroreflective tape regressions.

---

## local_mirror_layer

Nav2 `costmap_2d` plugin that subscribes to a source `OccupancyGrid` and accumulates selected cells into the host costmap via **max-merge**. Cells persist across host-costmap resizes — this is what lets the global costmap retain line/obstacle paste-ins as the robot drives the local rolling window past them.

| | |
|---|---|
| **Source** | `src/`, header in `include/local_mirror_layer/local_mirror_layer.hpp` |
| **Plugin class** | `local_mirror_layer::LocalMirrorLayer` (base `nav2_costmap_2d::Layer`) |
| **Plugin descriptor** | `local_mirror_layer.xml` (loaded by `nav2_costmap_2d` via `pluginlib`) |
| **Build** | `ament_cmake` |
| **Loaded by** | `nav2_params_camera.yaml` — listed in the global costmap's `plugins:` array |

> **Heads up:** Plugin, not node. The accumulate-on-resize behavior is the load-bearing part — competition runs depend on the global costmap not losing line obstacles as the local rolling window scrolls.

In the active camera profile there are two important mirror paths:

- `local_mirror_layer` consumes `/local_costmap/costmap`, masks `/line_costmap` and `/lidar_line_costmap`, and mirrors only lethal PCA obstacle seeds. This prevents cones from receiving a second copy of local soft inflation before global inflation.
- `line_memory_mirror_layer` consumes `/line_costmap` directly and mirrors exact lethal tape cells plus the camera-line soft halo after the global inflation layer. This makes Smac Lattice treat tape as a physical footprint-blocking obstacle while avoiding double-inflated local halos.

---

## map_padder

Pads the SLAM map outward so the robot, path, goal, or obstacles can live outside the explored region. Seed-and-flood: marks tiles around SLAM data + robot + goal + plan, expands by one ring of 8-neighbors, lethal everywhere else.

| | |
|---|---|
| **Provides** | Node `map_padder_node` (`map_padder/map_padder_node.py`) |
| **Subscribes** | `/map` (`OccupancyGrid`), `/goal_pose` (`PoseStamped`), `/plan` (`Path`) |
| **Publishes** | `/map_padded` (`OccupancyGrid`) |
| **Default params** | `tile_size_m: 1.0`, `output_resolution: 0.10`, plus topic remaps |
| **Build** | `ament_python` |

> **Heads up:** Launched alongside `slam_toolbox` in `slam.launch.py`. Nav2's global costmap consumes `/map_padded` — **not** raw `/map`.

---

## odom_handler

Wheel-odometry driver. Reads encoder counts over serial, integrates differential-drive kinematics, and publishes wheel odometry on `/odom`. It does **not** publish `odom → base_link` in the active build; `ekf_local` owns that TF.

| | |
|---|---|
| **Executables** | `wheel_odometry` (raw poller, no ROS) and `wheel_odometry_publisher` (the actual ROS node `wheelodom_publisher`) |
| **Subscribes** | `/encoders` (`autonav_interfaces/msg/Encoders`) |
| **Publishes** | `/odom` (`nav_msgs/msg/Odometry`) for `ekf_local` |
| **Constants** | `wheel_base_ = 0.6858 m`, `wheel_radius_ = 0.12946 m`, `ticks_per_revolution_ = 81923` |
| **Vendored** | `serialib.{cpp,hpp}` (duplicated from `gps_handler`); `src/reference.py` is scratch |
| **Build** | `ament_cmake` |

> **Heads up:** This is the dead-reckoning source the local EKF fuses with the IMU gyros.

---

## pointcloud_to_laserscan

Vendored utility (Paul Bovbel, BSD) for `PointCloud2` ↔ `LaserScan` conversion. Submodule from upstream.

| | |
|---|---|
| **Provides** | Two nodes: `pointcloud_to_laserscan_node`, `laserscan_to_pointcloud_node` |
| **Build** | `ament_cmake` (vendored) |

> **Heads up: not currently used.** SLAM gets `LaserScan` natively from `sick_scan_xd`; the PCA grade detector consumes `PointCloud2` directly. Removal candidate if maintenance becomes a chore.

---

## sick_scan_xd

Vendored upstream SICK driver for the MultiScan-100 LiDAR. Talks UDP over Ethernet.

| | |
|---|---|
| **Launch we use** | `launch/sick_multiscan.launch.py` (invoked by `./config/run-lidar.sh`) |
| **Publishes** | `/scan_fullframe` (`sensor_msgs/msg/LaserScan`), `/cloud_all_fields_fullframe` (`sensor_msgs/msg/PointCloud2`) |
| **Network** | LiDAR `192.168.0.1`, Jetson `192.168.0.2` on `eno1` |
| **Args we pass** | `hostname:=192.168.0.1`, `udp_receiver_ip:=192.168.0.2`, `publish_frame_id:=lidar_footprint`, `tf_publish_rate:=0` |
| **Resilience** | Driver declares `required="true"` so ROS2 respawns it if it dies |
| **Build** | `ament_cmake` (vendored) |

> **Heads up:** Don't edit upstream code. See [`docs/SENSORS.md`](./SENSORS.md#sick-multiscan-100-lidar) for the full bringup story (network auto-config, recovery, troubleshooting).

---

## sim

**Retired Gazebo Classic package.** This package has `COLCON_IGNORE`; use [`igvc_competition_sim`](#igvc_competition_sim) for the active ROS Humble + Gazebo Fortress simulation.

| | |
|---|---|
| **Robot descriptions** | `description/custom_robot/` (mesh-based full robot, STL baseframe) and `description/tester_robot/` (simplified box geometry for fast tests) |
| **Sensor xacros** | `depth_camera.xacro`, `gps.xacro`, `ground_truth.xacro`, `lidar.xacro` |
| **Configs** | `config/ekf.yaml`, `config/nav2_params.yaml`, `config/view_bot.rviz` |
| **Launch** | `launch_sim.launch.py`, `rsp.launch.py`, plus a stray `launch/opencv_node.cpp` (misplaced source) |
| **Worlds** | `worlds/autonav_igvc_course.world`, `worlds/empty.world` |
| **Models** | IGVC road sections (`igvc_asphalt`, `igvc_straightroad`, `igvc_curvedroad`, `igvc_bendedroad`, `igvc_mirrorbendedroad`, `igvc_potholeroad`) plus a resized `construction_barrel` |
| **Build** | `ament_cmake` |

> **Heads up:** Kept around for reference / rollback, not in active use. It is intentionally ignored by colcon.

---

## slam

Navigation backbone. Owns the launch files, configs, and behavior trees that wire SLAM + EKF + Nav2 together. No nodes of its own.

### Overview

Routes `/scan_fullframe` (LiDAR) → `slam_toolbox` → `map → odom` correction; fuses SICK IMU + wheel odom in the local EKF for the smooth `odom → base_link` chain at 30 Hz. The same launch also converts PCA obstacle points into `/scan_pca_filtered` and `/scan_pca_filtered_clear` for Nav2's local obstacle layer.

| | |
|---|---|
| **Build** | `ament_cmake` (pure config + launch) |

### Launch files and configs

**Launch (`launch/`):**

| File | What it brings up |
|---|---|
| `slam.launch.py` | **Active.** `ekf_local` + delayed `slam_toolbox` + PCA PointCloud2-to-LaserScan converters + `map_padder` + a delayed `[GUI_READY] SLAM` ExecuteProcess. Also declares the `enable_gps_fusion` arg (default `false`) — flipping to `true` brings up `ekf_global` + `navsat_transform_node` inline |
| `nav.launch.py` | Nav2 standalone, `nav2_params_camera.yaml`, `bt_2.xml` |
| `nav_lc.launch.py` | Minimal lifecycle manager for loop-closure testing |
| `dual_ekf_navsat.launch.py` | Standalone variant of the global-EKF + `navsat_transform_node` path. Superseded for normal use by the gated branch inside `slam.launch.py`; kept for isolation testing |

**Configs (`config/`):**

| File | Role |
|---|---|
| `nav2_params_camera.yaml` | Active/default Nav2 params: `nav_center` base frame, Smac Lattice global planner, MPPI local controller, camera-line/PCA costmap memory |
| `nav2_params_lidar.yaml` | Opt-in Nav2 params for lidar-line/PCA costmap memory regressions |
| `nav2_params.yaml` | Older Nav2 (legacy) |
| `slam.yaml` | `slam_toolbox`; subscribes to `/scan_fullframe`, 0.10 m resolution, scan matching enabled |
| `ekf_local.yaml` | Local EKF: 30 Hz, world_frame=odom, fuses wheel odom + `/sick_scansegment_xd/imu_inflated` |
| `ekf_local_sim.yaml` | Sim variant of ekf_local |
| `ekf_global.yaml` | Global EKF (configured but currently disabled) |
| `dual_ekf_navsat_params.yaml` | Future GPS-fusion config |
| `mapper_params_online_async.yaml` | Legacy slam_toolbox mapper params; not loaded by the active `slam.launch.py` |

### Behavior trees and the GPS fusion path

| | |
|---|---|
| **Behavior trees** | `bt.xml`, `bt_2.xml`, `bt_nav.xml` (the latter is what `run-nav2.sh` loads via `default_bt_xml_filename`). `bt_nav.xml` wraps the planner in a `<GoalUpdater>` that consumes `/goal_update` — that's how `gps_handler_node` corrects the live goal mid-leg without canceling FollowPath |
| **Global EKF / navsat status** | `ekf_global` + `navsat_transform_node` are wired into `slam.launch.py` but gated on the `enable_gps_fusion` launch arg (default `false`). Today, `slam_toolbox` publishes `map → odom` directly; `gps_handler_node` runs its own heading EKF and reads `/local_ekf/odom`, so GPS goals work without `ekf_global` in the loop |

> **Heads up:** The global EKF was observed publishing a frozen pose at startup, which made GPS goals worse rather than better — that's why `enable_gps_fusion` defaults off. Re-enabling it requires fixing the frozen-pose seed first. `gps_handler_node` is the live GPS path either way.

---

## zed_components

The actual ROS2 component library — wraps the Stereolabs ZED SDK into composable components (camera, depth, IMU, point cloud). The engine. Other `zed_*` packages are glue.

| | |
|---|---|
| **Build** | `ament_cmake` (vendored) |

> **Heads up:** Vendored via submodule, pinned to v5.2.0 / SHA `506e047`. **Do not edit.** **Do not run `git submodule update --remote`** — see [`docs/SENSORS.md`](./SENSORS.md#zed-submodule-pin) for the lock procedure.

---

## zed_debug

Dev tool — loads `zed_components` in a single C++ process for debugging instead of via the component container.

| | |
|---|---|
| **Build** | `ament_cmake` (vendored) |

> **Heads up:** Bundled with the wrapper. Safe to ignore unless you're debugging the camera.

---

## zed_ros2

Empty meta-package. Just declares the other `zed_*` packages as deps so apt/rosdep can install the bundle.

| | |
|---|---|
| **Build** | `ament_cmake` (vendored) |

> **Heads up:** Glue only. Safe to ignore.

---

## zed_wrapper

Standard runtime entry point for the ZED — loads `zed_components` in a single process and provides `zed_camera.launch.py`. This is what `./config/run-zed.sh` invokes.

| | |
|---|---|
| **Launch** | `launch/zed_camera.launch.py` |
| **Args we pass** | `camera_model:=zed2i`, `publish_tf:=false`, `publish_map_tf:=false`, `ros_params_override_path:=…/zed_override.yaml` |
| **Default configs** | `config/common_stereo.yaml`, `config/zed2i.yaml` (overridden by `bringup/config/zed_override.yaml`) |
| **Build** | `ament_cmake` (vendored) |

> **Heads up:** Tied to the pinned `zed_components`. Respect the submodule lock — see [`docs/SENSORS.md`](./SENSORS.md#zed-submodule-pin).
