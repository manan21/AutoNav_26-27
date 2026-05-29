# IGVC Fortress Simulation

The active Gazebo simulation target is `igvc_competition_sim`, built for ROS 2
Humble plus Gazebo Fortress. The older Gazebo Classic `sim` and `autonav_sim`
packages are intentionally ignored by colcon.

## Purpose

This simulation is meant to test the robot stack, not a parallel simulator
stack. The launch file starts the same AutoNav detection, Nav2, behavior tree,
custom costmap layers, GPS waypoint action server, and analysis topics used by
the robot.

Gazebo provides the course scene, robot dynamics target, and a rendered RGB-D
camera. `igvc_camera_bridge` relays that camera into the same ZED topics the
real camera line detector consumes. `igvc_sensor_harness` publishes the rest of
the robot-facing sensor contract:

- `/zed/zed_node/rgb/color/rect/image`
- `/zed/zed_node/rgb/color/rect/camera_info`
- `/zed/zed_node/depth/depth_registered`
- `/cloud_all_fields_fullframe`
- `/scan_fullframe`
- `/scan_pca_filtered_points`
- `/gps_fix`
- `/odom`
- `/local_ekf/odom`
- `/map_padded`
- `/joint_states`
- `/autonomous_mode`

The default line source is camera detection against plain white tape. The
SICK-style cloud is still available for PCA obstacle testing and opt-in
retroreflective/lidar-line regression work.

## Course

The compact course source is:

```bash
isaac_ros-dev/src/igvc_competition_sim/config/igvc_competition_compact.yaml
```

It includes:

- 10-20 ft lane widths.
- 3-inch plain white boundary tape plus an internal no-cross line.
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

The runner starts Gazebo Fortress, bridges `/clock`, `/cmd_vel`, Gazebo
odometry, and the RGB-D camera, launches the robot stack, records a bag, sends the configured
mission through `/navigate_to_waypoint`, and saves the live monitor score.

Useful environment overrides:

- `COURSE_CONFIG=/path/to/course.yaml`
- `RUN_DIR=/path/to/output`
- `MISSION_TIMEOUT_SEC=300`
- `GROUND_TRUTH_PCA=true`
- `LINE_DETECTION_MODE=camera` (default), `ground_truth`, or `lidar`
- `NAV2_PARAMS=/path/to/nav2_params_lidar.yaml` when using `LINE_DETECTION_MODE=lidar`
- `GAZEBO_SERVER_ONLY=false` to request the Gazebo GUI on a machine with a display
- `LAUNCH_GAZEBO=false` for harness-only debugging
- `LAUNCH_BRIDGE=false` when using a custom bridge

Line-source modes:

- `camera`: bridges Gazebo RGB-D into ZED topics, runs the CUDA camera line detector, and uses `nav2_params_camera.yaml`.
- `ground_truth`: publishes sampled course tape directly on `/line_points` to isolate Nav2 planning/control from camera perception.
- `lidar`: runs the SICK RSSI lidar-line detector for legacy retroreflective-tape regressions; pair it with `nav2_params_lidar.yaml`.

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
