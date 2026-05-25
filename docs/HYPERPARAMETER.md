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

## Quick index

| System | File | What it controls |
|---|---|---|
| Velocity envelope | `nav2_paramsv2.yaml` (velocity_smoother, FollowPath) | Robot speed cap, accel limits |
| Behavior Tree | `bt_nav.xml` | Replan cadence, recovery durations, retries |
| Phase B decorator | `nav2_paramsv2.yaml` (bt_navigator) | Cancel-restart suppression on same-path replans |
| Path planning | `nav2_paramsv2.yaml` (controller_server) | Progress checker, goal tolerance |
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

The effective cmd_vel cap is `min(DWB max_vel_x, velocity_smoother.max_velocity[0])`. Raising one without the other doesn't change behavior — raise both together.

| Parameter | Status | Default | Effect | Tune direction |
|---|---|---|---|---|
| `velocity_smoother.max_velocity[0]` | ⚠️ | `0.25` m/s (lab) / `0.50` (outdoor C.1) | Hard cap on forward `/cmd_vel.linear.x` | Higher → faster, watch for chatter past ~0.50 outdoors before EKF tuning |
| `velocity_smoother.min_velocity[0]` | 🟡 | `-0.25` / `-0.50` | Hard cap on reverse | Pair with max_velocity |
| `velocity_smoother.max_accel[0]` | 🟡 | `2.5` m/s² | Smoother linear accel cap | Higher → snappier ramp-up. Must equal `acc_lim_x` below. |
| `velocity_smoother.max_decel[0]` | 🟡 | `-2.5` m/s² | Smoother linear decel cap | Pair with max_accel |
| `controller_server.controller_frequency` | 🟡 | `20.0` Hz | DWB tick rate | Higher → more responsive, more CPU |
| `controller_server.FollowPath.max_vel_x` | 🟡 | `1.5` m/s | DWB's own upstream forward cap | Effective if you raise the smoother past this |
| `controller_server.FollowPath.min_speed_theta` | ⚠️ | `0.45` rad/s | Min DWB yaw-rate sample | Keeps pure turns above observed drivetrain deadband; lowering can reintroduce zero-RPM twitching |
| `controller_server.FollowPath.max_vel_theta` | 🟡 | `0.65` rad/s | Max yaw rate | Raised from 0.40 — lower starves GPS heading bootstrap |
| `controller_server.FollowPath.acc_lim_x` | 🟡 | `2.5` m/s² | DWB linear accel for trajectory sampling | **Must match `velocity_smoother.max_accel`** — DWB chatters at every accel ramp if they disagree |
| `controller_server.FollowPath.acc_lim_theta` | 🟡 | `0.9` rad/s² | DWB angular accel for trajectory sampling | Higher → snappier turns, overshoot risk |
| `controller_server.FollowPath.sim_time` | 🟡 | `1.2` s | DWB trajectory horizon | Lower (`0.8`) reduces over-commit when paired with cap raise (PHASEC.5) |

---

## Behavior tree (`isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml`)

| Parameter | Line | Status | Default | Effect |
|---|---|---|---|---|
| `RateController hz` | 33 | 🟡 | `1.0` | Replan cadence | Phase A.1: lowered 2 → 1 Hz to reduce cancel-restart count; Phase B decorator absorbs leftover same-path replans |
| `GoalBender bend_distance` | 59 | 🟡 | `0.8` m | Forward-bend intermediate distance | Only fires when `goal+path both behind robot` |
| `GoalBender angle_threshold` | 59 | 🟡 | `1.57` rad (90°) | Behind-robot trigger angle | Loose default |
| `GoalBender bend_angle` | 59 | 🟡 | `1.05` rad (60°) | Forward-bend offset | |
| `ComputePathRecovery number_of_retries` | 67 | 🟡 | `3` | Planner retries inside the rate-controlled subtree | |
| `ComputePathRecovery Wait wait_duration` | 81 | 🟡 | `0.1` s | Planner inter-retry wait | Phase A.4: lowered 0.3 → 0.1 |
| `FollowPathRecovery number_of_retries` | 94 | 🟡 | `2` | DWB retries before escalating | |
| `ClearCostmapAroundRobot reset_distance` | 112 | 🟡 | `1.0` m | FollowPath recovery clear radius | Phase A.5: replaces full local wipe — preserves line obstacles |
| `BackUp backup_dist` | 118 | 🟡 | `0.10` m | Distance reversed during BackUp recovery | Shortened so a mid-flight recovery drains in ~2 s |
| `BackUp backup_speed` | 118 | 🟡 | `0.05` m/s | BackUp speed | Slow by design — blind reverse |
| `DriveOnHeading speed` | 120 | 🟡 | `0.1` m/s | gradient_escape forward speed | |
| `DriveOnHeading time_allowance` | 120 | 🟡 | `15.0` s | gradient_escape budget | |
| `Wait wait_duration` (RoundRobin) | 127 | 🟡 | `1.0` s | Wait between recovery rounds | Phase A: lowered 5 → 1 so behavior_server drains fast |
| `NavigateRecovery number_of_retries` | (outer) | 🟡 | `999` | Outer recovery loop budget | Effectively infinite — by design (pure Phase A baseline). Re-cap if a watchdog is added |

---

## Phase B decorator (`nav2_paramsv2.yaml` under `bt_navigator.ros__parameters`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `path_significantly_changed.rms_threshold_m` | 🟢 | `0.10` m | First-N-pose RMS delta below which FollowPath cancel-restart is suppressed | Higher = less replanning chatter, but may delay reaction to genuine path changes. Live-tune: `ros2 param set /bt_navigator path_significantly_changed.rms_threshold_m 0.15` |
| `path_significantly_changed.compare_n_poses` | 🟢 | `10` | How many leading poses contribute to the RMS check | Higher = longer horizon for change detection (~0.5 m at 5 cm planner pose spacing). Raise to 20-30 if distant-obstacle reroutes look too slow |

---

## Path planning + goal checker (`nav2_paramsv2.yaml`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `controller_server.progress_checker.movement_time_allowance` | 🟡 | `15.0` s | Stuck-declare timeout when robot moves < `required_movement_radius` | Phase A.2: raised 8 → 15 to cover transient line-cross inflation |
| `controller_server.progress_checker.required_movement_radius` | 🟡 | `0.1` m | Minimum motion to count as "not stuck" within timeout | |
| `controller_server.general_goal_checker.xy_goal_tolerance` | 🟡 | `0.25` m | XY tolerance for goal-reached | Tightened from default 0.5 m |
| `controller_server.general_goal_checker.yaw_goal_tolerance` | 🟡 | `6.28319` rad (2π) | Yaw tolerance | Effectively disabled — only XY position counts |

---

## Costmap + inflation (`nav2_paramsv2.yaml`)

| Parameter | Status | Default | Effect |
|---|---|---|---|
| `local_costmap.local_costmap.update_frequency` | 🟡 | `15.0` Hz | Local costmap regeneration rate | |
| `local_costmap.local_costmap.publish_frequency` | 🟡 | `10.0` Hz | Local costmap broadcast rate | |
| `local_costmap.local_costmap.resolution` | 🟡 | `0.05` m | Local cell size | DWB uses this for narrow-gap precision; coarsening reduces fidelity |
| `local_costmap.local_costmap.width` / `height` | 🟡 | `6` m × `6` m | Rolling window size | **Must equal 2× `map_padder.local_window_radius_m`** so global mirrors local cleanly |
| `local_costmap.obstacle_layer.mark_scan.obstacle_max_range` | 🟡 | `2.5` m | Lidar marking range | Short — line obstacles only mark when close |
| `local_costmap.obstacle_layer.clear_scan.raytrace_max_range` | 🟡 | `25.0` m | Lidar clearing raytrace range | Long — clears all the way to SICK reach |
| `local_costmap.lidar_line_layer.observation_persistence_ms` | 🟡 | `7000` ms | Short LiDAR-line memory | Bridges detector dropouts / near-field blind spot, then clears under 10 s |
| `local_costmap.lidar_line_layer.inscribed_radius` | 🟡 | `0.36` m | High-cost band around LiDAR line points | Makes footprint overlap with tape costly without making the full halo lethal |
| `local_costmap.lidar_line_layer.inflation_radius` | 🟡 | `0.90` m | Outer LiDAR-line halo | Keeps visible gates passable while discouraging close line approaches |
| `local_costmap.inflation_layer.cost_scaling_factor` | 🟡 | `3.0` | Exponential decay of inflation cost | Lower → wider effective stay-away. Pair with global |
| `local_costmap.inflation_layer.inflation_radius` | 🟡 | `1.10` m | Max distance from obstacle for inflation | Pair with global |
| `global_costmap.global_costmap.update_frequency` | 🟡 | `3.0` Hz | Global regenerate rate | Conservative — global is mostly the paste from local |
| `global_costmap.global_costmap.resolution` | 🟡 | `0.10` m | Global cell size | Matches SLAM output resolution |
| `global_costmap.inflation_layer.cost_scaling_factor` | 🟡 | `3.0` | Same as local | **Must equal local** — controller and planner must agree on clearance |
| `global_costmap.inflation_layer.inflation_radius` | 🟡 | `1.10` m | Same as local | **Must equal local** |
| `global_costmap.line_layer.observation_persistence_ms` | 🟡 | `0` (indefinite) | How long line cells persist in global | `0` = never expire. Lower for forgetting old lines |

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
| `success_radius_m` | 🟢 | `0.5` m | Goal-reached arrival distance | Tighter than the 1.0 m default |
| `nav2_goal_hz` | 🟢 | `1.0` Hz | Goal republish cadence | Internal `gps_ekf` runs at GPS sample rate; this is just the submit rate |
| `feedback_hz` | 🟢 | `2.0` Hz | Action feedback rate | |
| `gps_stale_timeout_s` | 🟢 | `5.0` s | GPS outage timeout | |
| `tf_timeout_s` | 🟢 | `0.5` s | TF lookup timeout (map→odom) | |

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
| Autonomous | `wheel_speed_mps × 40 × stepSize × grade_mult` | `0.25 m/s × 40 × 10 × 1.0 = 100 = 10 %` |
| Manual | `joy_stick × speed × stepSize × grade_mult` | full stick × `10 × 10 × 1.0 = 100 = 10 %` |

`stepSize = 10` and `speed = 10` are deliberately matched so manual full-stick equals the 0.25 m/s autonomous cap — calibration parity.

### Motor controller build-time constants (`motor_controller.hpp`)

| Constant | Status | Default | Effect |
|---|---|---|---|
| `stepSize` | 🛠️ | `10` | move()-arg → per-mille RoboteQ throttle multiplier | Don't change without retuning Phase D + manual speed |
| `speed` | 🛠️ | `10` | Initial manual "gear" (full-stick scalar) | Set to `10` (was `11`) to match autonomous 0.25 m/s cap. Bumpers ±1 from here |

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
| `grade_comp_enabled` | 🟢 | `true` | Master gate. Set false to disable Phase D | |
| `imu_topic` | 🟢 | `/sick_scansegment_xd/imu_inflated` | IMU source | Switch to `/zed/zed_node/imu/data` if SICK path is unavailable |
| `imu_a_fwd_sign` | 🟢 | `+1.0` | Sign convention for accel.x (nose-up = positive) | Flip to `-1.0` if directionality is inverted on a swapped mount |
| `grade_comp_max_deg` | 🟢 | `10.0`° | Pitch angle at which max effect is reached | Beyond ±max_deg, multiplier holds at the bound |
| `grade_comp_max_uphill_pct` | 🟢 | `2.0` | Max boost — multiplier cap is `1 + this` (= 3.0×) | Raise to 3.0 if 15 % grade with 20 lb still stalls |
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

1. `ros2 param set /control_node grade_comp_enabled true` (default is true after the YAML flip; this is for re-enabling after disabling)
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
#                       min_velocity[0]: -0.50 -> -0.80
# Pair-edit if accel ramps look sluggish:
#   max_accel[0]: 2.5 -> 3.5 (smoother)
#   acc_lim_x:    2.5 -> 3.5 (DWB — must match smoother)
#   sim_time:     1.2 -> 0.8 (DWB — reduces over-commit)
```

### Tune Phase B decorator live

```bash
# Make the decorator more tolerant of GPS-driven goal jitter:
ros2 param set /bt_navigator path_significantly_changed.rms_threshold_m 0.15

# Or extend the comparison horizon to catch farther-out reroutes:
ros2 param set /bt_navigator path_significantly_changed.compare_n_poses 20
```

### Calibrate Phase D on a tilt block

```bash
# Already enabled by default. To raise the uphill boost cap if a payload
# test on the ramp still slows the robot:
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
- Per-pair tuning: smoother and DWB share `max_accel` ↔ `acc_lim_x` and `max_velocity` ↔ `max_vel_x`. Local and global costmaps share `cost_scaling_factor` ↔ `inflation_radius`. Always pair-edit these.
- Phase D is bounded by construction — the worst case is `[1 - max_downhill_pct, 1 + max_uphill_pct]` × baseline throttle, NaN-guarded, IMU-timeout-fallback to `1.0`. Comfort range is in the YAML defaults; widen the bounds only after a tilt-block bench test confirms motor current is in spec at the new bounds.
