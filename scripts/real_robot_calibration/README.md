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
./run_remote.sh record_manual_full_course --run-name manual_course_slow_pass_1
```

High-speed profiles require an explicit acknowledgement:

```bash
./run_remote.sh straight_speed_ladder_high --allow-high-speed
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
- `straight_speed_ladder_low`: straight 0.25, 0.5, and 1 mph commands.
- `straight_speed_ladder_high`: straight 2 and 3 mph commands; requires `--allow-high-speed`.
- `accel_stop_response`: 0.5 and 1 mph step/stop response.
- `accel_stop_response_high`: 2 mph step/stop response; requires `--allow-high-speed`.
- `in_place_yaw_ladder`: ±0.3, ±0.6, and ±1.0 rad/s in-place turns.
- `arc_ladder`: 0.5 and 1 mph arcs at ±0.3 and ±0.6 rad/s.
- `arc_ladder_high`: 2 mph arcs at ±0.3 rad/s; requires `--allow-high-speed`.
- `ramp_ladder`: 0.25, 0.5, and 1 mph straight ramp passes.

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
~/bags/practice_course/<profile>_<YYYYMMDD_HHMMSS>/bag
```

Each run directory also contains:

- `run_metadata.txt`
- `profiles.yaml`
- `topics.txt`
- `topic_list_at_start.txt`
- `missing_topics_at_start.txt`
- `rosbag_record.log`
- `command_profile.log` for scripted profiles

The suite runs inside tmux by default so Wi-Fi drops do not kill the bag. Detach
with `Ctrl-b d`.

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
