# Gazebo Canonicalization Plan

## Objective
Make the Gazebo IGVC simulation on `auto_camera` match the real robot, real
competition constraints, and real-world sensor behavior closely enough that
planning, path following, perception, recovery, dynamics, ramp handling, obstacle
avoidance, line avoidance, and GPS navigation improvements transfer to the real
robot.

This file is the durable plan for context compactions. Keep it current whenever
new analysis changes the direction.

## Data Sources
Use only these real-world runs unless the user explicitly expands the set.

Dynamics bags under:

```text
/Users/cole/autonav_bags/jetson_practice_course_20260529_211047
```

- `arc_ladder_1`
- `in_place_yaw_ladder_1`
- `straight_distance_drift_low_2`

Full perception bags under:

```text
/Users/cole/autonav_bag_backups
```

- `manual_course_rerun_5`
- `manual_course_rerun_7`
- `camera_line_detection_rerun`
- `camera_line_detection_rerun_2feet_stationary`

Phone videos under:

```text
/Users/cole/Downloads
```

- `arc_ladder_1.MOV`
- `in_place_yaw_ladder_1.MOV`
- `straight_distance_drift_low_2.MOV`
- `manual_course_rerun_5.MOV`
- `manual_course_rerun_7.MOV`
- `camera_line_detection_rerun.MOV`
- `camera_line_detection_rerun_two_feet_stationary.MOV`

Physical measurement for `straight_distance_drift_low_2`:

- forward distance: `30 ft 9 in` = `9.3726 m`
- right drift: `5 ft 3/16 in` = `1.5288 m`
- policy: use forward distance for odom sanity; treat lateral drift as
  slope-contaminated unless corroborated by arc/yaw bags.

## No-GPU Work
Run these analyses on the laptop/VMs before using an NVIDIA GPU.

- Inventory every selected bag and video: duration, size, topics, message
  counts, missing topics, run metadata, and git commit.
- Decode dynamics bags:
  - `/cmd_vel`, `/odom`, `/local_ekf/odom`, `/encoders`, `/motor_speed`, IMU.
  - Estimate commanded-vs-observed speed, yaw response, angular asymmetry,
    path length, stop settling, odom scale, and repeatability.
- Decode perception bags:
  - `/line_detection/diagnostics`, `/line_detection/line_pixels`,
    `/line_points`, `/line_costmap`, `/lines_pointcloud`.
  - Determine whether the camera detector sees tape in 2-D, whether 3-D line
    points are published, and whether costmaps receive line obstacles.
- Decode obstacle/costmap/planning evidence:
  - `/scan_pca_filtered*`, `/local_costmap/costmap`, `/global_costmap/costmap`,
    `/plan`, `/unsmoothed_plan`.
  - Look for obstacle persistence, premature clearing, plan-through-line,
    plan-through-obstacle, and fake narrow-gap behavior.
- Sync bags to videos:
  - use run names stated in the video plus the hand wave visible in camera and
    lidar topics.
  - record candidate bag timestamps and video offsets in `GAZEBO_WORK_LOG.md`.
- Update simulation config only when the evidence supports it. Do not tune away
  slope effects or bad data-quality artifacts.

## GPU-Deferred Work
Do this after a Linux NVIDIA GPU environment is available.

- Run the actual Gazebo camera-rendering path headless.
- Re-run the CUDA line detector on rendered tape and verify `/line_points` is
  non-empty.
- Compare real ZED tape pixel width/brightness/depth to the simulated camera.
- Run `autoresearch/evaluate.py` Tier 1, Tier 2, and Tier 3 after line detection
  is verified in sim.
- Add GPU-loop findings back to `GAZEBO_WORK_LOG.md`.

## Acceptance Criteria
Simulation is not considered trustworthy until all of these are true:

- Footprint, nav center, costmap footprint, and scorer footprint match.
- Real and simulated command response agree within documented tolerances for
  straight, in-place yaw, and arc tests.
- Camera line detection in sim produces comparable raw pixels, projected line
  points, and line costmap cells to real bags.
- PCA obstacle detections persist in the global costmap until raytrace-cleared.
- Camera line detections persist in the global costmap until camera-confirmed
  clear.
- Global plans avoid the padded robot footprint intersecting lines, obstacles,
  walls, potholes, and non-existent narrow gaps.
- Autoresearch score gates reject any line crossing, obstacle hit, failure to
  finish, or speed-rule violation.

## Current Execution Commands
Use the ROS22 Lima VM for bag access plus pure-Python bag decoding:

```bash
limactl shell autonav-ros22 -- bash -lc '
  cd /Users/cole/code/git/AutoNavB &&
  python3 scripts/analyze_gazebo_canon_offline.py \
    --output-json /tmp/gazebo_canon_offline_report.json \
    --output-md /tmp/GAZEBO_CANON_OFFLINE_REPORT.md
'
limactl copy autonav-ros22:/tmp/gazebo_canon_offline_report.json \
  /Users/cole/code/git/AutoNavB/docs/gazebo_canon_offline_report.json
limactl copy autonav-ros22:/tmp/GAZEBO_CANON_OFFLINE_REPORT.md \
  /Users/cole/code/git/AutoNavB/docs/GAZEBO_CANON_OFFLINE_REPORT.md
```

The `/Users/cole/code/git` mount is read-only inside `autonav-ros22`, so write
outputs to `/tmp` and copy them back from the host.

Use the Gazebo VM for live sim checks after data paths are mounted or the GPU
environment is ready:

```bash
limactl shell autonav-gazebo-sim
```
