# IGVC Fortress Simulation

The active Gazebo simulation target is `igvc_competition_sim`, built for ROS 2
Humble plus Gazebo Fortress. The older Gazebo Classic `sim` and `autonav_sim`
packages are intentionally ignored by colcon.

## Purpose

This simulation is meant to test the robot stack, not a parallel simulator
stack. The launch file starts the same AutoNav detection, Nav2, behavior tree,
custom costmap layers, GPS waypoint action server, and analysis topics used by
the robot-side lidar-line regression work.

Gazebo provides the course scene and robot dynamics target. The
`igvc_sensor_harness` publishes the robot-facing sensor contract:

- `/cloud_all_fields_fullframe`
- `/scan_fullframe`
- `/scan_pca_filtered_points`
- `/gps_fix`
- `/odom`
- `/local_ekf/odom`
- `/map_padded`
- `/joint_states`
- `/autonomous_mode`

The SICK-style cloud keeps the existing first-return lidar-line model for
reflective tape and pothole discs, while cylinder obstacles feed the PCA
obstacle path.

## Course

The compact course source is:

```bash
isaac_ros-dev/src/igvc_competition_sim/config/igvc_competition_compact.yaml
```

It includes:

- 10-20 ft lane widths.
- Curved boundary tape plus an internal no-cross line.
- Legal 5 ft passages and narrower decoy geometry.
- Barrels/posts, two 2 ft white-circle potholes, and a ramp below 15% grade.
- A four-leg mission using `/navigate_to_waypoint`, with the first local leg
  giving the GPS EKF enough motion before GPS waypoint legs.
- First-44-ft speed-check metadata and live scoring stations.

Regenerate the SDF after changing the YAML:

```bash
cd isaac_ros-dev/src/igvc_competition_sim
python3 -m igvc_competition_sim.generate_world \
  --course-config config/igvc_competition_compact.yaml \
  --output worlds/igvc_competition_compact.sdf
```

## Run

Build the workspace first:

```bash
cd isaac_ros-dev
colcon build
source install/setup.bash
```

Run the full test:

```bash
cd isaac_ros-dev/src/igvc_competition_sim
./Run_IGVC_COMPETITION_FORTRESS_TEST.command
```

The runner starts Gazebo Fortress, bridges `/clock`, `/cmd_vel`, and Gazebo
odometry, launches the robot stack, records a bag, sends the configured
mission through `/navigate_to_waypoint`, and saves the live monitor score.

Useful environment overrides:

- `COURSE_CONFIG=/path/to/course.yaml`
- `RUN_DIR=/path/to/output`
- `MISSION_TIMEOUT_SEC=300`
- `GROUND_TRUTH_PCA=true`
- `GAZEBO_SERVER_ONLY=false` to request the Gazebo GUI on a machine with a display
- `LAUNCH_GAZEBO=false` for harness-only debugging
- `LAUNCH_BRIDGE=false` when using a custom bridge

## Pass/Fail

`igvc_course_monitor` publishes `/igvc_sim/score` and `/igvc_sim/fail`. Current
hard failures are:

- Footprint crossing boundary tape, dashed tape, or internal no-cross lines.
- Footprint contact with obstacles or pothole discs.
- Leaving the legal ramp corridor while on the ramp.
- First 44 ft average speed below 1 mph.
- Speed above 5 mph.
- Stop/blocking interval over 60 s.

The bag still needs the same detailed post-run inspection used for lidar-line
regressions before treating a simulated pass as physical-robot evidence.
