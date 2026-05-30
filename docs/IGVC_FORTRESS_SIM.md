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

## Distributed Laptop + Jetson Mode

Use this mode when Gazebo runs on the laptop/ROS VM and the spare Jetson runs
the robot stack. This keeps Gazebo rendering and physics off the Jetson while
still exercising the Jetson CPU/GPU with Nav2, MPPI, camera-line CUDA
projection, PCA, costmaps, BT plugins, and command generation.

Network baseline:

- Mac/laptop Ethernet: `10.66.0.1/24`
- Spare Jetson Ethernet: `10.66.0.2/24`
- Gazebo Lima VM Ethernet: `10.66.0.3/24` on the direct Jetson Ethernet
  bridge.
- SSH alias: `jetson-spare-eth`
- Recommended ROS graph: `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`
- Confirm the link is gigabit before testing: `1000baseT <full-duplex>` on the
  laptop and `Speed: 1000Mb/s` on the Jetson.

For the current Mac/Lima setup, the Gazebo VM is `autonav-gazebo-sim`. Its
`~/.lima/autonav-gazebo-sim/lima.yaml` should include both the normal bridged
network and the direct Jetson Ethernet bridge:

```yaml
networks:
- lima: bridged
- lima: jetsoneth
```

The VM should assign the Jetson-side interface `10.66.0.3/24`; on the current
machine this is handled by the VM-local systemd unit
`autonav-jetsoneth-ip.service`.

Before running camera-line tests, confirm the generated SDF world contains the
Gazebo Sensors system. Without it, Gazebo publishes `/clock` and odometry but
does not publish `/igvc_sim/zed/*` camera topics:

```bash
grep -n "ignition-gazebo-sensors-system" \
  isaac_ros-dev/src/igvc_competition_sim/worlds/igvc_competition_compact.sdf
```

Run the Jetson stack inside the `koopa-kingdom` container:

```bash
ssh jetson-spare-eth
cd ~/AutoNav_25-26
ROS_DOMAIN_ID=42 AUTONAV_CONTAINER_GUI=0 ./env/docker/run-container.sh --no-attach
docker exec -it -u admin \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e AUTONAV_REPO=/autonav \
  -e ROS_WS=/autonav/isaac_ros-dev \
  koopa-kingdom \
  /bin/bash -lc 'cd /autonav/isaac_ros-dev/src/igvc_competition_sim && ./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command'
```

Run Gazebo and the Gazebo bridge on the laptop/ROS VM:

```bash
cd isaac_ros-dev/src/igvc_competition_sim
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command
```

Quick link validation:

```bash
# VM -> Jetson
limactl shell autonav-gazebo-sim ping -c 2 10.66.0.2

# Jetson -> VM
ssh jetson-spare-eth 'ping -c 2 10.66.0.3'

# ROS camera topics visible from inside the Jetson container
ssh jetson-spare-eth 'docker exec -u admin \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  koopa-kingdom \
  bash -lc "source /opt/ros/humble/setup.bash && \
    source /autonav/isaac_ros-dev/install/setup.bash && \
    timeout 4s ros2 topic hz /igvc_sim/zed/image"'
```

Role split:

- Laptop/VM: Gazebo Fortress, `ros_gz_bridge`, `/clock`, rendered RGB-D camera,
  Gazebo odometry, course monitor. The generated world runs physics at 100 Hz
  so distributed `/clock` traffic does not flood DDS.
- Jetson: `igvc_camera_bridge`, lightweight `igvc_odom_bridge` for 50 Hz
  `/odom`, `/local_ekf/odom`, and `/tf`, `igvc_sensor_harness` for simulated
  lidar/GPS/map support, CUDA camera line detector, PCA detector,
  pointcloud-to-laserscan converters, GPS waypoint action server, robot state
  publisher, Nav2, MPPI, custom BT plugins, and calibrated
  `/cmd_vel -> /cmd_vel_gazebo`.

Do not publish odom/TF from both sides in distributed mode. The VM sim-only
script disables `launch_odom_bridge`, and the Jetson stack script runs
`launch_odom_bridge:=true publish_harness_odom_tf:=false` so Nav2 sees one
monotonic 50 Hz base transform stream.

This mode intentionally moves raw RGB-D camera topics over the Ethernet link.
Use wired gigabit Ethernet or better; the Jetson USB gadget link is only
100 Mbps in this setup and is not appropriate for full-rate raw camera/depth.

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
