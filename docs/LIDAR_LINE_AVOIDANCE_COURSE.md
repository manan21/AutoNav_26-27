# Lidar Line Avoidance Test Course

This document defines the repeatable indoor course for testing lidar retroreflective line detection, PCA obstacle avoidance, Nav2 path following, and physical tape clearance.

Current course revision: the right-side passable gap has been widened to match the IGVC AutoNav minimum corridor width requirement. The passable gap between the perpendicular tape obstacle and the DOT cone is `5 ft` (`1.524 m`), and the perpendicular tape extends `0.63 m` rightward from the left-side tape.

## Purpose

The robot start position is marked on the floor so each run can use the same physical geometry. The test is intended to show whether the robot:

- Detects the retroreflective tape lines.
- Converts the detected lines into Nav2 obstacles/costs.
- Plans a valid path around the perpendicular tape line.
- Follows the planned path without overlapping tape or the cone.
- Reaches a 2.0 m relative forward goal from the marked start.

## Coordinate Convention

Use the starting lidar sensor position as the physical measurement reference unless otherwise noted.

- `+x`: forward from the robot start.
- `+y`: left of the robot.
- `-y`: right of the robot.
- Distances below are floor measurements.

If reconstructing the full course in robot/odom coordinates, account for the lidar-to-wheel-axis offset because the long left-side tape starts at the robot wheel axis, which is behind the lidar sensor.

## Course Geometry

### Long Left-Side Tape Barrier

- Material: retroreflective tape.
- Orientation: parallel to the robot's starting heading.
- Location: `0.50 m` left of the front lidar sensor path.
- Length: `3.20 m`.
- Start: aligned with the robot wheel axis near the back of the robot.
- Role: continuous left-side barrier along the test trajectory.

In the lidar-start coordinate convention, the tape centerline is approximately:

- `y = +0.50 m`
- `x` extends forward for `3.20 m`, starting from the wheel-axis-aligned point behind the lidar reference.

### Perpendicular Tape Obstacle

- Material: retroreflective tape.
- Orientation: perpendicular to the left-side tape, extending rightward from it.
- Forward location: `0.905 m` in front of the robot's lidar sensor.
- Length: `0.63 m`.
- Start: begins at the long left-side tape line.
- Direction: extends to the robot's right from the left-side tape.

Using the lidar-start coordinate convention and assuming the tape is square to the left barrier, the perpendicular tape is approximately:

- `x = +0.905 m`
- `y` spans from `+0.50 m` to `-0.13 m`

The robot should avoid this tape by driving around the right-hand end, not by crossing over it.

### Cone Obstacle

- Object: large orange DOT construction cone with retroreflective tape.
- Location: to the right of the perpendicular tape obstacle.
- Gap: `5 ft` (`1.524 m`) between the right end of the perpendicular tape and the cone.
- Role: forms the right-side boundary of the passable gap.

The robot must pass through the `5 ft` (`1.524 m`) gap between the right end of the perpendicular tape and the cone without touching either obstacle.

For bag analysis, the right end of the perpendicular tape is approximately `y = -0.13 m` in the lidar-start frame. If the cone's left edge is treated as the right boundary of the measured `5 ft` gap, that boundary is approximately `y = -1.654 m`. With Nav2's padded footprint half-width of roughly `0.44 m`, a no-yaw centerline corridor is approximately:

- Tape-side clearance: `nav_center_y < -0.57 m`.
- Cone-side clearance: `nav_center_y > -1.21 m`.
- Preferred nominal corridor center: around `nav_center_y = -0.89 m`.

These are analysis targets, not hard controller commands. Yaw, localization error, costmap resolution, and obstacle thickness reduce the usable clearance, so a global plan that only clips around the tape end near `y = -0.3 m` to `-0.5 m` is still likely to be infeasible for DWB's footprint critic.

## Test Command

Send a relative goal `2.0 m` forward from the robot's marked start. Use the nav-center-relative goal convention when available so commanded travel and measured travel are consistent.

Before sending the relative goal, the human operator must ensure the robot is already in autonomous mode. Do not have the test runner publish a synthetic X-button press: autonomous mode is a toggle, and an automated X press can accidentally switch a robot that is already in AUTO back to MANUAL.

Preferred command from inside the robot ROS environment:

```bash
isaac_ros-dev/config/send_goal.sh -r 2.0 0 0
```

Do not publish the same test goal through both `/goal_pose` and a direct `/navigate_to_pose` action client. `/goal_pose` already triggers Nav2's `NavigateToPose` path and also seeds `map_padder`; sending both creates overlapping NavigateToPose goals and can make the status stream show simultaneous `ABORTED` and `EXECUTING` goals.

Expected high-level behavior:

- The robot detects the perpendicular tape before reaching it.
- Nav2 plans around the right-hand end of the perpendicular tape.
- The robot passes through the `5 ft` (`1.524 m`) gap between the tape and cone.
- The robot does not overlap the long left barrier, perpendicular tape, or cone.
- The robot reaches the forward goal without requiring manual costmap clearing.

## Required Runtime State

Before sending the goal:

- Nav2 is running and active.
- Lidar line detection is running.
- PCA obstacle detection is running.
- SLAM/localization is running.
- The human operator has confirmed the robot is in autonomous mode.
- The local and global costmaps are not carrying stale obstacles from a previous run.

## Data To Record

Record a rosbag for each run. Include at least:

Use `ros2 bag record --include-hidden-topics` so Nav2 action status topics are actually captured. For live controller tests, do not record `/cloud_all_fields_fullframe` unless raw lidar debugging is required; it can add enough load to affect Nav2 timing.

- `/tf`
- `/tf_static`
- `/local_ekf/odom`
- `/odom`
- `/cmd_vel`
- `/cmd_vel_nav`
- `/encoders`
- `/motor_speed`
- `/autonomous_mode`
- `/lidar_line_points`
- `/lidar_line_costmap`
- `/lidar_line_detection/diagnostics`
- `/scan_pca_filtered_points`
- `/terrain/grade_map`
- `/pca/surface_normal`
- `/local_costmap/costmap`
- `/local_costmap/costmap_raw`
- `/global_costmap/costmap`
- `/global_costmap/costmap_raw`
- `/plan`
- `/local_plan`
- `/navigate_to_pose/_action/status`
- `/follow_path/_action/status`
- `/compute_path_to_pose/_action/status`

## Analysis Checklist

After every run, stop the rosbag cleanly with SIGINT, confirm `ros2 bag info` shows the expected topics and duration, and analyze both the live monitor output and bagged telemetry before reporting a pass/fail result.

When analyzing the bag in the robot ROS environment, use:

```bash
python3 scripts/analyze_lidar_line_bag.py /path/to/bag
```

The analyzer reports the `nav_center` displacement, command response, local-plan dropouts, action status results, detected line-point clearance against the configured footprint, and `/lidar_line_costmap` clearing behavior.

- Whether `/lidar_line_points` matches the measured tape geometry.
- Whether `/lidar_line_costmap` marks the perpendicular and parallel tape lines before the robot reaches them.
- Whether `/scan_pca_filtered_points` marks the cone.
- Whether the global plan routes through the `5 ft` (`1.524 m`) gap.
- Whether the global plan and local plan put `nav_center` inside the approximate centerline corridor, not merely around the tape endpoint.
- Whether the local plan stays outside the measured tape and cone boundaries.
- Whether the robot footprint overlaps the perpendicular tape, left-side tape, or cone at any point.
- Whether the robot reaches `2.0 m` forward travel from the marked start.
- Whether nonzero `/cmd_vel` commands produce matching encoder, RPM, and `/local_ekf/odom` motion.
- Whether any `No valid trajectories`, `Failed to make progress`, missed controller loops, stale transform warnings, or Nav2 action aborts coincide with stalls.

The test report should include:

- Bag path and `ros2 bag info` summary.
- Goal result status and final relative `nav_center` displacement.
- Maximum right/left lateral deviation while avoiding the tape.
- Closest observed tape points in robot frame near the footprint.
- A short conclusion: pass, fail due to line overlap, fail due to controller stall, or inconclusive due to operator/runtime setup.

## Pass Criteria

A clean pass requires:

- No physical overlap with retroreflective tape or cone.
- Successful Nav2 result for the `2.0 m` relative forward goal.
- Robot footprint remains inside the passable corridor and clears the perpendicular tape's right end.
- Lidar-line and PCA obstacle detections are present before avoidance behavior begins.
- Costmaps clear normally after the run or are explained by active detections still present in sensor data.
