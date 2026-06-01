# Hyperparameter Reference — AutoNav

A catalog of the tunable knobs across the autonomy stack: where each parameter lives, what it controls, which direction to tune, and what's dangerous to change. Read the **Safety legend** before tuning anything marked sensitive.

---

## Safety legend

| Marker | Meaning |
|---|---|
| 🟢 | Live-tunable. Safe to adjust via `ros2 param set …` mid-mission. |
| 🟡 | YAML-tunable. Edit, restart the stack — pair-tune with linked params. |
| 🔴 | **OSCILLATION-SENSITIVE.** Documented invariants. Read the inline rationale before touching; expect to break end-to-end behavior if changed without understanding the chain. |
| ⚠️ | **CONTROLLER-CHATTER-SENSITIVE.** Lower bound or pairing exists; lowering past the documented floor reproducibly reintroduces a tuning bug we've already paid for. |
| 🛠️ | Build constant. Edit the C++ / Python source, then rebuild (`colcon build --packages-select <pkg>`). |

When a row carries 🔴 or ⚠️, the comment block adjacent to the parameter in source is the source of truth — start there before changing the value.

---

## YAML configuration inventory

Every YAML file under `isaac_ros-dev/src/` that we own (vendored YAMLs
in `sick_scan_xd/` and `zed-ros2-wrapper/` are out of scope). Status
column shows whether the file is loaded by the live GUI launch path.

**Legend**
- ✅ **active** — loaded somewhere on the GUI launch path or by a node the GUI starts.
- ⚙️ **gated** — available from an active launch file, but disabled by default behind a launch argument.
- 🧪 **sim-only** — only loaded by the simulation launch files; ignored on the real robot.
- 📦 **legacy / unused** — file is present in the tree but no current launch loads it. Candidate for deletion.

| File | Status | Purpose | Loaded by |
|---|---|---|---|
| `slam/config/nav2_paramsv2.yaml` | ✅ active | Nav2 params (Smac Lattice, MPPI, costmaps, planner_server, bt_navigator, smoother, velocity_smoother) — the big one | `run-nav2.sh`, `slam.launch.py`, `nav.launch.py` |
| `slam/config/slam.yaml` | ✅ active | SLAM Toolbox node params: raw LiDAR scan, scan matching, map→odom publication | `slam.launch.py` |
| `slam/config/ekf_local.yaml` | ✅ active | Local EKF — wheel odom + IMU fusion, owns `odom → base_link` | `slam.launch.py` |
| `slam/config/ekf_global.yaml` | ⚙️ gated off by default | Global EKF — adds GPS XY fusion when `enable_gps_fusion:=true`; does not publish `map → odom` | `slam.launch.py` only when `enable_gps_fusion:=true` |
| `autonav_detection/config/line_detector.yaml` | ✅ active | Camera line detector thresholds, ROI, morphology | `detection.launch.py` (via `run-lines.sh`) |
| `autonav_detection/config/grade_detector.yaml` | ✅ active | PCA grade detector params (window, slope thresholds, PCA dims) | `detection.launch.py` (via `run-pca.sh`) |
| `autonav_detection/config/lidar_line_detector.yaml` | ✅ active when LiDAR line detect is launched | SICK RSSI/reflector tape detector, ground gate, clustering, segment completion | `detection.launch.py` (via `run-lidar-lines.sh`) |
| `control/config/node_params.yaml` | ✅ active | Control node — Phase D grade comp, manual speed gear, motor mapping | `control_dev.launch.py` (via `run-pre-slam.sh`) |
| `control/config/config_params.yaml` | ✅ active | Control node — additional param overlay | `control_dev.launch.py` |
| `bringup/config/zed_override.yaml` | ✅ active | ZED wrapper parameter overrides (resolution, TF suppression, depth mode) | `run-zed.sh` |
| `autonav-gui-hud/config/watched_topics.yaml` | ✅ active | GUI's live topic watchlist for status dots and live tick | `hud_node.py` |
| `autonav_automated_testing/config/testing_data_collection_setter.yaml` | ✅ active (testing) | t000_* automated test runner config | `t000_DAQ_MODE.launch.py`, `t000_AUTO_DAQ_MODE.launch.py` |
| `sim/config/ekf.yaml` | 🧪 sim-only | EKF params for Gazebo simulation | sim launch files |
| `sim/config/nav2_params.yaml` | 🧪 sim-only | Nav2 params for simulation (sim_time enabled, different costmap layers) | sim launch files |
| `slam/config/ekf_local_sim.yaml` | 📦 legacy / unused | Predecessor sim variant of `ekf_local.yaml`. No active loader. Delete candidate. | — |
| `slam/config/nav.yaml` | 📦 legacy / unused | Predecessor Nav2 config. No active loader. Delete candidate. | — |
| `slam/config/nav_minimal.yaml` | 📦 legacy / unused | Stripped-down Nav2 variant. No active loader. Delete candidate. | — |
| `slam/config/nav_defaults.yaml` | 📦 legacy / unused | Default Nav2 variant. No active loader. Delete candidate. | — |
| `slam/config/nav2_params.yaml` | 📦 legacy / unused | Predecessor to `nav2_paramsv2.yaml`. Only referenced by `run-nav.sh` (also unused — GUI uses `run-nav2.sh`). Delete candidate. | `run-nav.sh` only |
| `slam/config/mapper_params_online_async.yaml` | 📦 legacy / unused | Older slam_toolbox mapper params; `slam.launch.py` explicitly uses `slam.yaml` instead. | — |
| `slam/config/dual_ekf_navsat_params.yaml` | 📦 legacy / unused | navsat_transform_node params. `dual_ekf_navsat.launch.py` exists but is not included by any GUI-path launch file. Delete candidate (or wire up the launch if the GPS handler's heading bootstrap should use it). | `dual_ekf_navsat.launch.py` (not invoked from GUI path) |
| `autonav_automated_testing/config/calibration_constants.yaml` | 📦 legacy / unused | No active loader. Delete candidate. | — |

If you delete any of the 📦 files, also delete the launch file or script that's its only caller (e.g. `run-nav.sh` ↔ `nav2_params.yaml`), or you leave a dangling reference behind.

---

## Quick index

| System | File | What it controls |
|---|---|---|
| Velocity envelope | `nav2_paramsv2.yaml` (velocity_smoother, MPPI `FollowPath`) | Robot speed cap, accel limits |
| Behavior Tree | `bt_nav.xml` | Replan cadence, recovery durations, retries |
| Global planning | `nav2_paramsv2.yaml` (planner_server) | Smac Lattice primitive file, lattice penalties, planning budget |
| Local control | `nav2_paramsv2.yaml` (controller_server `FollowPath`) | MPPI sampling, critics, goal tolerance |
| Costmap / inflation | `nav2_paramsv2.yaml` (local_costmap, global_costmap) | Obstacle inflation, line layer persistence |
| SLAM | `slam.yaml` | Scan rate, match confidence, keyframes |
| EKF (local + global) | `ekf_local.yaml`, `ekf_global.yaml` | Sensor fusion config, TF ownership |
| GPS handler | `gps_handler_node.py` (constants + ROS params) | Goal republish cadence, heading resync thresholds |
| Map padder | `map_padder_node.py` (ROS params) | Corridor tile size, grid resolution |
| Control / motor | `node_params.yaml`, `motor_controller.hpp` | Phase D grade comp, manual speed gear |
| Phase D grade comp | `node_params.yaml` (control_node) | IMU-driven uphill boost / downhill damping |
| Joystick | `xbox.cpp` + `control.cpp` button handlers | Button mapping, deadband |

---

## Robot velocity envelope (`nav2_paramsv2.yaml`)

The effective forward cap is `min(MPPI FollowPath.vx_max, velocity_smoother.max_velocity[0])`. Raising one without the other does not change the robot's top speed, and raising both without retuning the MPPI critics can reintroduce start-stop chatter around frequent replans.

| Parameter | Status | Default | Effect | Tune direction |
|---|---|---|---|---|
| `velocity_smoother.max_velocity[0]` | ⚠️ | `0.50` m/s | Hard cap on forward `/cmd_vel.linear.x` | Higher → faster, watch for chatter past ~0.50 outdoors before EKF tuning |
| `velocity_smoother.min_velocity[0]` | 🟡 | `-0.25` m/s | Hard cap on reverse | Leave lower unless deliberately testing faster reverse/recovery motion |
| `velocity_smoother.max_accel[0]` | 🟡 | `1.2` m/s² | Smoother linear accel cap | Higher → snappier ramp-up; retune with MPPI samples and chatter checks. |
| `velocity_smoother.max_decel[0]` | 🟡 | `-1.2` m/s² | Smoother linear decel cap | Pair with max_accel |
| `controller_server.controller_frequency` | 🟡 | `20.0` Hz | MPPI control tick rate | Higher → more responsive, more CPU |
| `controller_server.FollowPath.vx_max` | ⚠️ | `0.50` m/s | MPPI forward sampling cap | Pair with `velocity_smoother.max_velocity[0]` |
| `controller_server.FollowPath.vx_std` | ⚠️ | `0.25` m/s | MPPI forward sampling spread | Must be wide enough for MPPI to explore the intended cap; `0.12` still behaved like a ~0.20 m/s controller |
| `controller_server.FollowPath.vx_min` | 🟡 | `0.0` m/s | MPPI reverse sampling cap | Current local controller is forward-only; reverse is handled by `breadcrumb_reverse` |
| `controller_server.FollowPath.vy_max` | 🔴 | `0.0` m/s | Lateral sampling cap | Must remain zero for differential drive |
| `controller_server.FollowPath.wz_std` | 🟡 | `0.45` rad/s | MPPI angular sampling spread | Higher → samples sharper yaw corrections, higher overshoot risk |
| `controller_server.FollowPath.wz_max` | 🟡 | `0.65` rad/s | MPPI angular sampling cap | Higher → more aggressive turns, higher overshoot risk |
| `controller_server.FollowPath.time_steps × model_dt` | 🟡 | `56 × 0.05 = 2.8` s | MPPI rollout horizon | Longer sees farther but costs CPU and can over-commit |
| `controller_server.FollowPath.batch_size` | 🟡 | `1200` | Samples per MPPI tick | Higher → better local search, more CPU |

---

## Behavior tree (`isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml`)

| Parameter | Line | Status | Default | Effect |
|---|---|---|---|---|
| `RateController hz` | 43 | 🟡 | `3.0` | Replan cadence | Matches 3 Hz global costmap updates so newly detected tape reaches the global plan within ~17 cm at 0.50 m/s |
| `GoalBender bend_distance` | 67 | 🟡 | `0.8` m | Forward-bend intermediate distance | Fires when the path leads behind the robot and the goal/path context requires a turnaround |
| `GoalBender angle_threshold` | 69 | 🟡 | `1.57` rad (90°) | Behind-robot trigger angle | Loose default |
| `GoalBender bend_angle` | 69 | 🟡 | `1.05` rad (60°) | Forward-bend offset | |
| `ComputePathRecovery number_of_retries` | 88 | 🟡 | `8` | Planner retries inside the rate-controlled subtree | Gives SLAM/TF/costmap warm-up up to ~4 s before heavy recovery |
| `PathFootprintSafe footprint_padding` | 96 | 🟡 | `0.05` m | Runtime global-plan footprint gate | Rejects paths whose rectangular `nav_center` footprint overlaps lethal raw global cells |
| `ComputePathRecovery Wait wait_duration` | 113 | 🟡 | `0.5` s | Planner inter-retry wait | Paired with 8 retries for warm-up patience |
| `PathSignificantlyChanged force_update_period_s` | 27 | 🟡 | `0.5` s | Maximum age for the active controller path | Keeps MPPI path updates fresh without replacing the FollowPath action on every 3 Hz same-corridor replan |
| `FollowPathRecovery number_of_retries` | 151 | 🟡 | `2` | MPPI retries before escalating | |
| `ClearCostmapAroundRobot reset_distance` | 157 | 🟡 | `1.0` m | FollowPath recovery clear radius | Local-only clear; preserves global line/obstacle memory |
| `BackUp backup_dist` | 179 | 🟡 | `0.10` m | Distance reversed during BackUp recovery | Shortened so a mid-flight recovery drains in ~2 s |
| `BackUp backup_speed` | 179 | 🟡 | `0.05` m/s | BackUp speed | Slow by design — blind reverse |
| `DriveOnHeading speed` | 180 | 🟡 | `0.1` m/s | `gradient_escape` forward speed | |
| `DriveOnHeading time_allowance` | 181 | 🟡 | `15.0` s | `gradient_escape` budget | |
| `Wait wait_duration` (RoundRobin) | 190 | 🟡 | `1.0` s | Wait between recovery rounds | Short enough to keep recovery responsive |
| `NavigateRecovery number_of_retries` | 28 | 🟡 | `999` | Outer recovery loop budget | Effectively infinite — by design. Re-cap if a watchdog is added |

---

## Active FollowPath Churn Gate Params (`nav2_paramsv2.yaml` under `bt_navigator.ros__parameters`)

`PathSignificantlyChanged` is active in `bt_nav.xml`. It filters 3 Hz global replans before `FollowPath`: route-geometry changes pass through immediately, and same-corridor refreshes are forwarded at least every bounded TTL. This avoids action-server churn while preserving fresh MPPI paths through lidar-line route changes.

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `path_significantly_changed.rms_threshold_m` | 🟡 | `0.15` m | RMS route-shape delta that triggers a fresh FollowPath action |
| `path_significantly_changed.compare_n_poses` | 🟡 | `20` | Number of normalized path samples used for route comparison |
| `path_significantly_changed.max_point_delta_m` | 🟡 | `0.30` m | Single-sample route-shape delta that triggers a fresh FollowPath action |
| `path_significantly_changed.start_delta_threshold_m` | 🟡 | `0.25` m | Large start jump threshold; ordinary progress along the same path should not churn FollowPath |
| `path_significantly_changed.length_delta_threshold_m` | 🟡 | `0.50` m | Route-length delta that triggers a fresh FollowPath action |
| `path_significantly_changed.force_update_period_s` | 🟡 | `0.5` s | Bounded same-corridor refresh interval for MPPI |

---

## Planning, control, and goal checker (`nav2_paramsv2.yaml`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `planner_server.expected_planner_frequency` | 🟡 | `20.0` Hz | Planner-server expected rate | BT still gates global replans at 3 Hz |
| `planner_server.GridBased.plugin` | 🔴 | `nav2_smac_planner/SmacPlannerLattice` | Footprint-aware SE2 lattice planner | Replaced SmacPlanner2D so global paths account for rectangular footprint geometry |
| `planner_server.GridBased.lattice_filepath` | 🔴 | `/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json` | Differential-drive lattice primitive set | Must match global costmap `resolution: 0.05` |
| `planner_server.GridBased.cost_penalty` | 🟡 | `5.0` | Cost sensitivity during lattice search | Higher avoids soft costs more aggressively |
| `planner_server.GridBased.rotation_penalty` | 🟡 | `5.0` | Penalty for rotation-heavy lattice paths | Lower if the robot needs more precise maneuvers; too low made rotation shortcuts attractive |
| `controller_server.FollowPath.plugin` | 🔴 | `nav2_mppi_controller::MPPIController` | Active local controller | Requires `ros-humble-nav2-mppi-controller` in the runtime image |
| `controller_server.FollowPath.CostCritic.consider_footprint` | 🔴 | `true` | MPPI collision scoring uses full footprint | Must stay true for tape/cone clearance |
| `controller_server.FollowPath.CostCritic.collision_cost` | 🟡 | `1000000.0` | Cost assigned to colliding trajectories | High by design so MPPI does not choose line/cone overlap |
| `controller_server.FollowPath.PathAlignCritic.max_path_occupancy_ratio` | 🟡 | `0.12` | How much path occupancy the align critic tolerates | Tune only with bags; too strict can reject useful local deviations |
| `controller_server.progress_checker.movement_time_allowance` | 🟡 | `15.0` s | Stuck-declare timeout when robot moves < `required_movement_radius` | Phase A.2: raised 8 → 15 to cover transient line-cross inflation |
| `controller_server.progress_checker.required_movement_radius` | 🟡 | `0.1` m | Minimum motion to count as "not stuck" within timeout | |
| `controller_server.general_goal_checker.xy_goal_tolerance` | 🟡 | `0.25` m | XY tolerance for goal-reached | Tightened from default 0.5 m |
| `controller_server.general_goal_checker.yaw_goal_tolerance` | 🟡 | `6.28319` rad (2π) | Yaw tolerance | Effectively disabled — only XY position counts |

---

## Costmap + inflation (`nav2_paramsv2.yaml`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `local_costmap.local_costmap.update_frequency` | 🟡 | `15.0` Hz | Local costmap regeneration rate | |
| `local_costmap.local_costmap.publish_frequency` | 🟡 | `15.0` Hz | Local costmap broadcast rate | |
| `local_costmap.local_costmap.resolution` | 🟡 | `0.05` m | Local cell size | MPPI and line-layer clearance depend on this; coarsening reduces narrow-gap fidelity |
| `local_costmap.local_costmap.width` / `height` | 🟡 | `6` m × `6` m | Rolling window size | **Must equal 2× `map_padder.local_window_radius_m`** so global mirrors local cleanly |
| `local_costmap.local_costmap.footprint_padding` | 🟡 | `0.03` m | Local controller footprint margin | Global padding is intentionally larger (`0.05` m) so unsafe paths are rejected before MPPI |
| `local_costmap.obstacle_layer.mark_scan.obstacle_max_range` | 🟡 | `2.5` m | Lidar marking range | Short — line obstacles only mark when close |
| `local_costmap.obstacle_layer.clear_scan.raytrace_max_range` | 🟡 | `25.0` m | Lidar clearing raytrace range | Long — clears all the way to SICK reach |
| `local_costmap.lidar_line_layer.observation_persistence_ms` | 🟡 | `-1` | Manual-clear-only LiDAR-line memory | Temporary tape-test behavior so the robot cannot forget a detected line while paused |
| `local_costmap.lidar_line_layer.inscribed_radius` | 🟡 | `0.10` m | High-cost band around LiDAR line points | Wider than exact point cells, narrow enough not to box in the start pose |
| `local_costmap.lidar_line_layer.inflation_radius` | 🟡 | `0.80` m | Outer LiDAR-line halo | Softly pushes MPPI/Smac away from sparse tape detections |
| `local_costmap.inflation_layer.cost_scaling_factor` | 🟡 | `4.0` | Exponential decay of PCA obstacle inflation cost | Pair with global |
| `local_costmap.inflation_layer.inflation_radius` | 🟡 | `0.85` m | Max distance from PCA obstacle for stock inflation | Matched to global so cones and tape are not artificially imbalanced |
| `global_costmap.global_costmap.update_frequency` | 🟡 | `3.0` Hz | Global regenerate rate | Conservative — global is mostly the paste from local |
| `global_costmap.global_costmap.resolution` | 🔴 | `0.05` m | Global cell size | Must match the 5 cm Smac Lattice primitive file |
| `global_costmap.global_costmap.footprint_padding` | 🟡 | `0.05` m | Planner/path gate footprint margin | Rejects paths that are centerline-clear but unsafe for the real body |
| `global_costmap.local_mirror_layer.min_occupied_value_to_mirror` | 🟡 | `100` | Mirrors only lethal PCA obstacle seeds from local costmap | Prevents local soft inflation from being copied into global and inflated again |
| `global_costmap.local_mirror_layer.exclude_topics` | 🟡 | `["/line_costmap", "/lidar_line_costmap"]` | Masks line-layer cells from obstacle memory mirroring | Prevents lidar lines from entering global through the wrong layer |
| `global_costmap.lidar_line_memory_mirror_layer.allow_decrease` | 🟡 | `false` | Global LiDAR-line memory clearing policy | Manual-clear-only for the current tape-test behavior |
| `global_costmap.inflation_layer.cost_scaling_factor` | 🟡 | `4.0` | Shared decay for PCA/cone and lidar tape seeds | **Must equal local** unless intentionally testing a planner/controller mismatch |
| `global_costmap.inflation_layer.inflation_radius` | 🟡 | `0.85` m | Shared global inflation radius | Applied once after PCA obstacle seeds and exact lidar tape seeds enter global |

---

## SLAM (`slam.yaml`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `slam_toolbox.scan_topic` | 🟡 | `/scan_fullframe` | Lidar source for SLAM | Not the PCA-filtered scan |
| `slam_toolbox.mode` | 🟡 | `mapping` | SLAM mode | `localization` for replay against a saved map |
| `slam_toolbox.resolution` | 🟡 | `0.10` m | Map cell size | Matches global_costmap input |
| `slam_toolbox.minimum_time_interval` | 🟡 | `0.3` s | Scan rate limiter | Halve to `0.15` if inter-scan motion exceeds matcher search window at higher caps |
| `slam_toolbox.throttle_scans` | 🟡 | `2` | Drop every Nth scan | Keeps SLAM at ~5 Hz at 10 Hz SICK |
| `slam_toolbox.transform_publish_period` | 🟡 | `0.02` s (50 Hz) | map→odom TF broadcast rate | |
| `slam_toolbox.use_scan_matching` | 🔴 | `true` | Enable correlative scan matcher | **MUST stay true.** Part of the three-part oscillation invariant — false reintroduces "map catching up" snap |
| `slam_toolbox.link_match_minimum_response_fine` | 🔴 | `0.3` | Confidence floor for accepting a fine scan match | **DO NOT lower.** Below 0.3, weak matches fuse and snap map→odom forward, breaking GPS nav |
| `slam_toolbox.correlation_search_space_dimension` | 🟡 | `0.5` m | Search window side length | At ~0.5 m cap, inter-scan motion = 15 cm — comfortable headroom. Above 0.8 m/s, halve `minimum_time_interval` or raise this |
| `slam_toolbox.minimum_travel_distance` | 🟡 | `0.5` m | Keyframe insertion distance threshold | Prevents pose-graph bloat |
| `slam_toolbox.minimum_travel_heading` | 🟡 | `0.3` rad (~17°) | Keyframe insertion yaw threshold | |

**Three-part SLAM oscillation invariant** (all must hold simultaneously, see comments in `slam.yaml`):

1. `slam_toolbox.use_scan_matching: true`
2. `ekf_local.odom0_config[6]: false` (wheel-vx OFF) **AND** `ekf_local.imu0: /sick_scansegment_xd/imu_inflated`
3. `imu_cov_inflator` running (inflates raw SICK IMU covariance to realistic 0.01 rad²/s² on yaw rate)

Break any of those and the robot reintroduces map↔odom drift/stall oscillation.

---

## EKF local (`ekf_local.yaml`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `ekf_node.frequency` | 🟡 | `30.0` Hz | EKF update rate | Pair with `ekf_global` |
| `ekf_node.publish_tf` | 🔴 | `true` | Publish `odom → base_link` | **MUST stay true** — ekf_local OWNS this TF. Dual publishers cause race + jitter |
| `ekf_node.odom0_config[6]` (vx) | 🔴 | `false` | Wheel forward velocity fusion | **MUST stay false** — when enabled, EKF over-fuses velocity and bounces vs. slam_toolbox correction |
| `ekf_node.odom0_config[11]` (vyaw) | 🔴 | `true` | Wheel yaw-rate fusion | **MUST stay true** — cross-cancels gyro bias. Without it ~82°/run drift |
| `ekf_node.odom0_pose_rejection_threshold` | 🟡 | `5.0` σ | Mahalanobis outlier gate on wheel odom | Catches encoder glitches without rejecting normal motion |
| `ekf_node.imu0` | 🔴 | `/sick_scansegment_xd/imu_inflated` | IMU topic | **MUST use the inflated topic.** Raw SICK has zero covariance → over-fusion → yaw snap |
| `ekf_node.imu0_config[11]` (gyro yaw) | 🟡 | `true` | Fuse gyro yaw rate | Standard |
| `ekf_node.process_noise_covariance[5]` | 🟡 | `0.06` | Yaw process noise | Higher = less trust in motion model |
| `ekf_node.process_noise_covariance[11]` | 🟡 | `0.02` | Vyaw process noise | Wheel + gyro cross-check stabilizes |

## EKF global (`ekf_global.yaml`)

> **Note:** `ekf_global` + `navsat_transform_node` are gated by the `enable_gps_fusion` launch arg on `slam.launch.py`, which defaults to `false`. Live GPS waypoint navigation runs through `gps_handler_node` (reads `/local_ekf/odom`) without `ekf_global` in the loop. The parameters below only take effect when the operator launches with `enable_gps_fusion:=true`.

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `ekf_global.frequency` | 🟡 | `30.0` Hz | Global EKF update rate | |
| `ekf_global.publish_tf` | 🔴 | `false` | Publish `map → odom` | **MUST stay false** — slam_toolbox owns map→odom. Flipping to true puts dual publishers on the same edge |
| `ekf_global.odom1` (GPS) | 🟡 | `/odometry/gps` | GPS XY input topic | Via navsat_transform_node |
| `ekf_global.odom1_config` | 🟡 | `[true, true, false, …]` | Fuse only XY from GPS | Yaw from SLAM, velocity from wheels — by design |
| `ekf_global.odom1_pose_rejection_threshold` | 🟡 | `5.0` σ | Mahalanobis outlier gate on GPS | 5σ tolerates multipath spikes |

---

## GPS handler (`gps_handler_node.py`)

### Live-tunable ROS params

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `success_radius_m` | 🟢 | `0.5` m | Goal-reached arrival distance | Tighter than the 1.0 m default; per-goal override via `goal_msg.success_radius_m` |
| `nav2_goal_hz` | 🟢 | `1.0` Hz | Goal republish cadence | Internal `gps_ekf` runs at GPS sample rate; this is just the submit rate. Same value gates both `/goal_pose` (leg start) and `/goal_update` (in-mission corrections) |
| `feedback_hz` | 🟢 | `2.0` Hz | Action feedback rate | |
| `gps_stale_timeout_s` | 🟢 | `5.0` s | GPS outage timeout | |
| `tf_timeout_s` | 🟢 | `0.5` s | TF lookup timeout (map→odom) | |
| `map_frame` / `odom_frame` | 🟢 | `map` / `odom` | Frame names for TF lookups | |
| `next_hint_enabled` | 🟢 | `false` | Consume `/gps_waypoint/next_hint` for look-ahead on chained legs | |
| `hint_match_tolerance_m` | 🟢 | `0.5` m | Acceptance radius for matching a hint to the next leg | |
| `coldstart_bias_enabled` | 🟢 | `false` | Snap θ_offset on first GPS goal so the waypoint lands ahead of `base_link` | `run-gps.sh` passes `true` |
| `coldstart_theta_seed_variance_deg` | 🟢 | `45.0`° | Initial variance for the coldstart θ seed | Intentionally loose so the first real EKF heading update dominates immediately |

### File-edit constants (gps_handler_node.py header)

| Constant | Status | Default | Effect |
|---|---|---|---|
| `GOAL_REPUBLISH_HEARTBEAT_S` | 🛠️ | `0.2` s | Inner throttle gate on per-tick publish | Loose by design post-A.3 — gate is non-restrictive at 1 Hz tick |
| `GOAL_POSE_HEARTBEAT_S` | ⚠️🛠️ | `60.0` s | First-publish-per-leg cycle for /goal_pose | **DO NOT lower below 3 s** — bt_navigator full replan on every `/goal_pose`. Post-A.3 routes in-mission via `/goal_update` instead |
| `HEADING_RESYNC_THRESHOLD_DEG` | ⚠️🛠️ | `15.0`° | Yaw-drift threshold to trigger heading resync | **DO NOT lower** — 10° caused start-stop chatter; 15° lets θ oscillate around bias mean |
| `HEADING_RESYNC_COOLDOWN_S` | ⚠️🛠️ | `5.0` s | Minimum gap between resync events | DO NOT shorten — each resync causes a controller turn |
| `CANDIDATE_SMOOTH_ALPHA` | ⚠️🛠️ | `0.15` | EWMA on candidate-goal filter | Don't tune without watching `/gps_waypoint/debug` across a full mission |
| `BOOTSTRAP_*` constants | 🛠️ | various | Heading bootstrap thresholds | See file header §5 for tuning rationale |
| `HEALTH_DEGRADED_THETA_DEG` | 🛠️ | `5.0` | Threshold for DEGRADED health badge in GUI | |
| `HEALTH_FAIL_THETA_DEG` | 🛠️ | `15.0` | Threshold for FAIL health badge | |

---

## Map padder (`map_padder_node.py`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `tile_size_m` | 🟡 | `1.0` m | Geometric tile size for corridor discretization | |
| `output_resolution` | 🟡 | `0.10` m/cell | Output OccupancyGrid resolution | Match SLAM resolution |
| `local_window_radius_m` | 🟡 | `3.0` m | Half-side of local-costmap seed region | **MUST equal local_costmap width/2** so global mirrors local |

Design invariants (not tunables — described for context):

- Corridor bounding box grows monotonically. Once a tile is in the corridor, it stays.
- Cells in the cumulative corridor never become LETHAL even if the corridor shrinks later — prevents "eating away" map behind the robot.
- MAX_GRID_SIDE = 1600 cells × 0.10 m = 160 m maximum coverage. Plenty for an IGVC course.

---

## Control / motor (`node_params.yaml` + `motor_controller.hpp`)

### Motor command math

| Path | Per-mille throttle formula | Example |
|---|---|---|
| Autonomous | `wheel_speed_mps × 40 × stepSize × grade_mult` | `0.50 m/s × 40 × 10 × 1.0 = 200 = 20 %` |
| Manual | `joy_stick × speed × stepSize × grade_mult` | full stick × `10 × 10 × 1.0 = 100 = 10 %` |

`stepSize = 10` and `speed = 10` keep manual full-stick conservative. The outdoor AutoNav cap is now higher than manual default; raise manual `speed` only as a separate driver-control tuning decision.

Autonomous deadband compensation is applied after `cmd_vel` is converted to per-wheel motor arguments and after grade compensation. Exact zero remains zero; small same-direction traction commands are lifted so Nav2's valid low-speed turn requests do not sit below static friction. During forward/reverse translation, tiny opposite-sign inner-wheel commands are not boosted, which avoids turning small corrections into snap pivots. With defaults, `auto_deadband_min_motor_arg = 6.5`, so `6.5 × stepSize(10) = 65` per-mille RoboteQ throttle.

### Control node motor tunables (`node_params.yaml`)

| Parameter | Status | Default | Effect | Notes |
|---|---|---|---|---|
| `auto_deadband_comp_enabled` | ⚠️ | `true` | Enables autonomous-only minimum effective motor command | Does not affect manual joystick or exact zero stop commands |
| `auto_deadband_min_motor_arg` | ⚠️ | `6.5` | Minimum `motors.move()` argument for each nonzero wheel command | Raise toward `7.5` if the robot still twitches; lower toward `5.5` if turns become too abrupt |
| `auto_deadband_apply_below_motor_arg` | ⚠️ | `6.5` | Only commands below this magnitude are lifted | Keep equal to `auto_deadband_min_motor_arg` unless deliberately shaping the transition |
| `auto_curvature_cap_enabled` | 🟢 | `true` | Caps autonomous `angular.z` while driving forward | Keeps turn slowdown from creating high-curvature low-speed pivots |
| `auto_min_turn_radius_m` | 🟢 | `0.90` m | Minimum forward-driving turn radius in the control bridge | Lower permits sharper corrections; higher damps swinging more |

### Motor controller build-time constants (`motor_controller.hpp`)

| Constant | Status | Default | Effect |
|---|---|---|---|
| `stepSize` | 🛠️ | `10` | move()-arg → per-mille RoboteQ throttle multiplier | Don't change without retuning Phase D + manual speed |
| `speed` | 🛠️ | `10` | Initial manual "gear" (full-stick scalar) | Conservative manual default. Bumpers ±1 from here |

### Control safety gates

| Gate | Status | Default | Effect |
|---|---|---|---|
| Joy watchdog | 🛠️ | `0.5` s | No `/joy` for 0.5 s → motors zero (manual only) | |
| Bumper debounce | 🛠️ | `200` ms | Minimum interval between speed-gear changes | |
| `WHEEL_BASE` | 🛠️ | `0.6858` m | Differential-drive kinematics constant | Robot-physical, not a tuning knob |
| IMU subscription QoS | 🛠️ | `SensorDataQoS` (BEST_EFFORT) | Match imu_cov_inflator QoS | **DO NOT use RELIABLE** — silently drops every message |

---

## Phase D — gravity-vector grade compensation (`node_params.yaml` under `control_node.ros__parameters`)

All 10 params are live-tunable. Phase D applies in **both manual and autonomous** modes when `forward_command ≠ 0`. Multiplier is mathematically bounded to `[1 - max_downhill_pct, 1 + max_uphill_pct]` regardless of IMU input.

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `grade_comp_enabled` | 🟢 | `false` | Master gate. Set true to enable Phase D | Currently disabled by default — bench-tested 2026-05-18 |
| `imu_topic` | 🟢 | `/sick_scansegment_xd/imu_inflated` | IMU source | Switch to `/zed/zed_node/imu/data` if SICK path is unavailable |
| `imu_a_fwd_sign` | 🟢 | `+1.0` | Sign convention for accel.x (nose-up = positive) | Flip to `-1.0` if directionality is inverted on a swapped mount |
| `grade_comp_max_deg` | 🟢 | `10.0`° | Pitch angle at which max effect is reached | Beyond ±max_deg, multiplier holds at the bound |
| `grade_comp_max_uphill_pct` | 🟢 | `1.0` | Max boost — multiplier cap is `1 + this` (= 2.0×) | Raise to 2.0 (3.0× cap) if 15 % grade with 20 lb still stalls |
| `grade_comp_max_downhill_pct` | 🟢 | `0.30` | Max damping — multiplier floor is `1 - this` (= 0.70×) | Raise toward 0.65 if robot accelerates past smoother cap on descent |
| `grade_comp_deadband_deg` | 🟢 | `0.5`° | Level-ground noise filter | Raise to 0.75-1.0 if level-ground vibration causes chatter |
| `grade_comp_ramp_max_velocity_mps` | 🟢 | `0.30` m/s | **Autonomous-only.** Caps base linear velocity *before* boost when on an incline | Lower for tighter ramp safety; raise once outdoor tests prove the ramp can be taken faster |
| `grade_comp_imu_timeout_sec` | 🟢 | `0.5` s | If no IMU for this long, multiplier reverts to 1.0 | |
| `grade_comp_alpha` | 🟢 | `0.2` | EWMA smoothing coefficient on `a_fwd` | Lower → smoother but more lag; higher → noisier but faster |

### Direction-aware logic

Multiplier depends on the sign of `pitch_deg × forward_command`:

| Pitch | Motion | Relation to gravity | Multiplier |
|---|---|---|---|
| nose-up (`+`) | forward (`+`) | against | boost |
| nose-up (`+`) | reverse (`-`) | with | damp |
| nose-down (`-`) | forward (`+`) | with | damp |
| nose-down (`-`) | reverse (`-`) | against | boost |

This handles the backing-down-a-hill case correctly — gravity helps reverse motion, so damping fires.

### Bench-test calibration

1. `ros2 param set /control_node grade_comp_enabled true` (default is currently `false`; enable only for a deliberate tilt/ramp test)
2. Note level-ground wheel RPM at full-stick forward in manual mode (call it 100 %).
3. Tilt nose-up ~10° → wheels at ~300 % of level RPM (boost).
4. Tilt nose-down ~10° → wheels at ~70 % of level RPM (damping).
5. Push joystick *backward* on both tilt directions to verify the symmetric cases.

If directions are inverted, flip `imu_a_fwd_sign` live and re-test.

---

## Joystick (`xbox.cpp` + `control.cpp` handlers)

| Control | Index | Effect | Notes |
|---|---|---|---|
| Left stick Y | axes[1] | Left motor speed in tank-drive | Range ±1.0, deadband ±0.1 |
| Right stick Y | axes[3] | Right motor speed in tank-drive | Range ±1.0, deadband ±0.1 |
| X button (autonomous toggle) | buttons[3] | Toggle AUTO/MANUAL mode, publishes to `/autonomous_mode` | Rising-edge |
| Y button (costmap clear) | buttons[2] | Calls `/global_costmap/clear_entirely_global_costmap` | Rising-edge; testing aid |
| B button (stop) | buttons[1] | Zero motors | |
| Right bumper (speed up) | buttons[6] OR buttons[9] | `setSpeed(+1)` | 200 ms debounce |
| Left bumper (speed down) | buttons[7] OR buttons[10] | `setSpeed(-1)` | 200 ms debounce |

Bumper indexes are OR'd across two layouts ({6,7} normal / {9,10} wrong-container-mount) because xpad mapping differs between mounts. Trade-off: clicking a stick on a layout where {9,10} are stick clicks nudges the speed gear — non-critical UX cost.

---

## Common tuning workflows

### Raise the velocity cap before an outdoor mission

```bash
# After PHASE C.1 outdoor test was clean at 0.50, before C.2 try at 0.80:
# YAML edit (preferred — survives relaunch):
#   nav2_paramsv2.yaml: max_velocity[0]: 0.50 -> 0.80
#   Keep min_velocity[0] at -0.25 unless faster reverse is part of the test.
# Pair-edit the MPPI cap or the smoother will still clamp at the lower value:
#   FollowPath.vx_max: 0.50 -> 0.80
# Then retune in bags if chatter returns:
#   FollowPath.vx_std, time_steps/model_dt, PathAlignCritic/PathFollowCritic weights
#   velocity_smoother.max_accel/max_decel
```

### Tune the FollowPath churn gate

```bash
ros2 param set /bt_navigator path_significantly_changed.rms_threshold_m 0.15
ros2 param set /bt_navigator path_significantly_changed.compare_n_poses 20
ros2 param set /bt_navigator path_significantly_changed.force_update_period_s 0.5
```

### Calibrate Phase D on a tilt block

```bash
# Phase D is disabled by default. Enable it for a deliberate ramp test, then
# raise the uphill boost cap if a payload test still slows the robot:
ros2 param set /control_node grade_comp_max_uphill_pct 3.0    # cap = 4.0x

# To increase downhill damping if the robot still accelerates past cap:
ros2 param set /control_node grade_comp_max_downhill_pct 0.50  # floor = 0.50x

# Once tuned, persist back into node_params.yaml.
```

### Quick stop conditions

If the robot is misbehaving on the tilt block / ramp:

```bash
ros2 param set /control_node grade_comp_enabled false   # disables Phase D
ros2 param set /control_node grade_comp_ramp_max_velocity_mps 0.10  # caps autonomous ramp speed harder
```

---

## When in doubt

- 🔴 / ⚠️ parameters have inline comments at the source location. Read those before changing.
- The OSCILLATION-SENSITIVE three-part SLAM invariant must remain intact: `use_scan_matching: true`, EKF local using `imu_inflated`, and `imu_cov_inflator` running. Breaking any one reintroduces the canonical "map catching up" snap.
- Per-pair tuning: smoother and MPPI share the effective forward cap (`max_velocity[0]` ↔ `FollowPath.vx_max`). Local and global costmaps share `cost_scaling_factor` ↔ `inflation_radius`. Always pair-edit these unless you are deliberately testing a planner/controller mismatch.
- Phase D is bounded by construction — the worst case is `[1 - max_downhill_pct, 1 + max_uphill_pct]` × baseline throttle, NaN-guarded, IMU-timeout-fallback to `1.0`. Comfort range is in the YAML defaults; widen the bounds only after a tilt-block bench test confirms motor current is in spec at the new bounds.
