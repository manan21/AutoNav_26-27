# Real Robot Calibration Bag Suite

This suite records timestamped ROS bags on the Jetson and, for scripted
profiles, publishes repeatable `/cmd_vel` motion commands so real robot
behavior can be calibrated against the Gazebo sim.

Run the suite from the laptop:

```bash
cd ~/code/git/AutoNav_25-26/scripts/real_robot_calibration
./run_remote.sh --list
./run_remote.sh record_manual_full_course
./run_remote.sh straight_speed_ladder_low
./run_remote.sh straight_distance_drift_low
./run_remote.sh record_manual_full_course --run-name manual_course_slow_pass_1
./run_remote.sh camera_line_static_canon --run-name camera_line_static_canon_1
./run_remote.sh nav_debug_driveway_gap --run-name nav_debug_driveway_gap_1
./run_remote.sh manual_nav_shadow_course --run-name manual_nav_shadow_course_1
```

High-speed profiles require an explicit acknowledgement:

```bash
./run_remote.sh straight_speed_ladder_high --allow-high-speed
./run_remote.sh straight_distance_drift_high --allow-high-speed
./run_remote.sh arc_ladder_high --allow-high-speed
```

The default robot SSH target is `jetson`. Override it when needed:

```bash
./run_remote.sh straight_speed_ladder_low --robot jetson
```

## Operator Workflow

1. Bring up the robot stack normally.
2. Put one person on the controller and one person on the wireless e-stop.
3. Start phone video.
4. Run the chosen profile from this directory on the laptop.
5. When the script prints the run name, say the run name and current video time out loud.
6. Wave in front of the ZED camera and lidar for sync.
7. For scripted profiles, toggle AUTO with Xbox X only when the robot is pointed safely.
8. If the robot becomes unsafe, toggle AUTO off or hit the wireless e-stop. The command runner aborts when `/autonomous_mode` becomes false.

The scripts do not fake joystick input by default. Scripted profiles wait for
the real control node to publish `/autonomous_mode=true`, then command
`/cmd_vel`.

## Profiles

- `record_manual_full_course`: full perception/planning recording while you manually drive.
- `closed_loop_observe`: full recording while RViz/Nav2 controls the robot.
- `perception_static_sweep`: full recording for stationary or hand-crept views of tape, cones, ramp, shadows, and narrow gaps.
- `camera_line_static_canon`: strict static white-tape camera projection validation.
- `camera_line_motion_creep`: strict 0.25 mph approach over/near white tape.
- `nav_debug_driveway_gap`: strict autonomous Nav2 debug recording for tape/obstacle gaps, plans, costmaps, and recovery.
- `manual_nav_shadow_course`: strict manual-driven course run while Nav2 plans from an RViz goal in the background.
- `costmap_memory_yaw_sweep`: strict yaw sweep to test global costmap memory for lines and obstacles.
- `pca_obstacle_memory_manual`: strict manual PCA obstacle persistence/smearing capture; usually run with `--raw-lidar`.
- `gps_nav_observe`: strict GPS/RViz/Nav2 closed-loop observation.
- `straight_speed_ladder_low`: straight 0.25, 0.5, and 1 mph commands.
- `straight_speed_ladder_high`: straight 2 and 3 mph commands; requires `--allow-high-speed`.
- `straight_distance_drift_low`: odom-distance-gated straight runs, 3 m at 0.5 mph and 5 m at 1 mph.
- `straight_distance_drift_high`: odom-distance-gated straight runs, 6 m at 2 mph and 8 m at 3 mph; requires `--allow-high-speed`.
- `straight_distance_level_10m_1mph`: level-ground 10 m odom scale/drift test at 1 mph.
- `accel_stop_response`: 0.5 and 1 mph step/stop response.
- `accel_stop_response_high`: 2 mph step/stop response; requires `--allow-high-speed`.
- `in_place_yaw_ladder`: ±0.3, ±0.6, and ±1.0 rad/s in-place turns.
- `arc_ladder`: 0.5 and 1 mph arcs at ±0.3 and ±0.6 rad/s.
- `arc_ladder_high`: 2 mph arcs at ±0.3 rad/s; requires `--allow-high-speed`.
- `ramp_ladder`: 0.25, 0.5, and 1 mph straight ramp passes.
- `ramp_ladder_full_perception`: strict ramp dynamics plus perception transfer capture.

## Canon Field-Test Roadmap

Run the canon profiles in this order when collecting the remaining simulation
transfer bags. For every run: start phone video, say the run name, say the
video time, then wave in front of the ZED and lidar.

P0 mission-critical perception and planning:

1. `camera_line_static_canon`: stationary white tape at about 2 ft, 4 ft, and 8 ft; include diagonal tape plus sun/shadow if available. Confirm line pixels become `/line_points` and `/line_costmap` cells.
2. `camera_line_motion_creep`: slow 0.25 mph approach toward/near tape. Confirm motion does not collapse projection due to TF/depth sync failures.
3. `nav_debug_driveway_gap`: recreate tape plus wall/cone false-gap issue, start recording, then place the RViz goal. Confirm `/plan`, costmaps, line costmap, obstacle sources, and recovery behavior are recorded.
4. `manual_nav_shadow_course`: keep AUTO off/manual, start recording, place an RViz/Nav2 goal, then manually drive the course through ramp, cones, tape, legal gaps, and illegal narrow-gap cases. Confirm Nav2 plans and costmaps respond correctly while the operator supplies actual motion.

P1 costmap memory and obstacle persistence:

5. `costmap_memory_yaw_sweep`: place visible tape and a cone/wall in front, then yaw away and back. Confirm global costmap line/obstacle memory persists until properly cleared.
6. `pca_obstacle_memory_manual --raw-lidar`: manually drive past cones/walls as they enter, leave, and re-enter lidar view. Use this for PCA persistence and local-costmap smearing analysis.

P2 dynamics, ramp, and GPS transfer:

7. `straight_distance_level_10m_1mph`: run on level measured ground in both directions with separate run names.
8. `in_place_yaw_ladder`: rerun only after `/odom`, `/local_ekf/odom`, `/encoders`, and `/tf` are confirmed present.
9. `arc_ladder`: repeat on level ground for turn-radius and yaw-under-translation calibration.
10. `ramp_ladder_full_perception`: capture speed loss, IMU pitch/grade, perception continuity, and costmaps on a competition-style ramp.
11. `gps_nav_observe`: run GPS/RViz waypoint navigation over a small course section to capture GPS, plans, recovery, and average-speed behavior.

Speed conversions used by the profiles:

```text
0.25 mph = 0.111760 m/s
0.50 mph = 0.223520 m/s
1.00 mph = 0.447040 m/s
2.00 mph = 0.894080 m/s
3.00 mph = 1.341120 m/s
```

## Bag Location and Naming

Bags are written on the Jetson under:

```text
/autonav_bags/practice_course/<profile>_<YYYYMMDD_HHMMSS>/bag
```

`/autonav_bags` is a host-backed Docker mount created by
`env/docker/run-container.sh`, so bags survive robot power cycles, branch
switches, and `git reset --hard` in `AutoNav_25-26`. If the robot container was
started before that mount existed, the suite falls back to
`/autonav/logs/real_robot_calibration`, which is also persistent across power
cycles and normal git branch/reset operations. Native, non-container runs use
`~/autonav_bags/practice_course`. Override the root explicitly with
`--base-dir DIR` or `AUTONAV_CALIB_BASE_DIR`.

Each run directory also contains:

- `run_metadata.txt`
- `profiles.yaml`
- `topics.txt`
- `topic_list_at_start.txt`
- `missing_topics_at_start.txt`
- `rosbag_record.log`
- `command_profile.log` for scripted profiles
- `command_metrics.csv` for scripted profiles that compute run metrics, including straight-distance drift rows

The suite runs inside tmux by default so Wi-Fi drops do not kill the bag. Detach
with `Ctrl-b d`. On the real Jetson, recording runs inside the `koopa-kingdom`
container so custom AutoNav message types are available to `ros2 bag record`.
The preferred `/autonav_bags` root is mounted from the Jetson host and is
visible from the normal SSH shell as `~/autonav_bags`. The fallback
`/autonav/logs/real_robot_calibration` is visible from the normal SSH shell as
`~/AutoNav_25-26/logs/real_robot_calibration`.

## Stopping a Run

From the laptop:

```bash
./stop_remote.sh
```

Or from inside the tmux session, press `Ctrl-C`. Cleanup publishes zero
`/cmd_vel`, stops `ros2 bag record` with SIGINT, and runs `ros2 bag info`.

## Topic Profiles

`topics/dynamics_light.txt` is for speed, turning, and ramp dynamics. It avoids
high-bandwidth camera debug topics.

`topics/full_perception.txt` is for perception/planning runs. It records ZED
image/depth topics, line detector outputs, lidar/PCA outputs, costmaps, plans,
and hidden Nav2 action status topics. `/cloud_all_fields_fullframe` is excluded
by default because it can add enough load to perturb live behavior. Add it only
when explicitly needed:

```bash
./run_remote.sh perception_static_sweep --raw-lidar
```

`topics/canon_full.txt` is the stricter simulation-transfer profile. Profiles
using it set `strict_required_topics: true`, so the script aborts before
recording if required ZED, TF, odom, line, and costmap topics are missing.

## Important Safety Notes

- Do not run scripted profiles with an active Nav2 goal publishing competing `/cmd_vel`.
- Use a long clear straightaway for 2-3 mph tests.
- Use a large open area for arc and yaw tests.
- For ramp tests, align the robot manually before toggling AUTO.
- Turning AUTO off should abort scripted motion within the command runner; the physical e-stop remains the final safety authority.

## Local Dry Runs

Dry runs do not SSH or publish ROS commands:

```bash
./run_remote.sh straight_speed_ladder_low --dry-run
./run_remote.sh straight_speed_ladder_high --allow-high-speed --dry-run
```
