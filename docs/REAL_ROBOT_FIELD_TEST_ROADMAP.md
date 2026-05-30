# Real Robot Field-Test Roadmap

This roadmap lists the remaining bags needed to make Gazebo canon to the real
robot and competition course. Run the profiles from:

```bash
cd ~/code/git/AutoNavB/scripts/real_robot_calibration
```

For every run, start phone video, say the run name, say the video time, then
wave in front of the ZED and lidar.

## P0: Mission-Critical Perception And Planning

1. `camera_line_static_canon`
   - Command: `./run_remote.sh camera_line_static_canon --run-name camera_line_static_canon_1`
   - Setup: robot stationary facing white tape at about 2 ft, 4 ft, and 8 ft; include diagonal tape plus sun/shadow if available.
   - Success: line pixels, `/line_points`, and `/line_costmap` are all nonzero.

2. `camera_line_motion_creep`
   - Command: `./run_remote.sh camera_line_motion_creep --run-name camera_line_motion_creep_1`
   - Setup: robot aimed safely toward/near tape for a 0.25 mph approach.
   - Success: `/line_points` remains nonzero during motion and does not collapse from TF/depth sync failures.

3. `nav_debug_driveway_gap`
   - Command: `./run_remote.sh nav_debug_driveway_gap --run-name nav_debug_driveway_gap_1`
   - Setup: recreate tape plus wall/cone false-gap issue; start recording before placing the RViz goal.
   - Success: bag contains plans, local/global costmaps, line costmap, obstacle sources, TF, odom, action status, and recovery behavior.

## P1: Costmap Memory And Obstacle Persistence

4. `costmap_memory_yaw_sweep`
   - Command: `./run_remote.sh costmap_memory_yaw_sweep --run-name costmap_memory_yaw_sweep_1`
   - Setup: visible tape plus cone/wall in front of robot.
   - Success: global costmap line/obstacle memory persists while rotating away and back until properly cleared.

5. `pca_obstacle_memory_manual`
   - Command: `./run_remote.sh pca_obstacle_memory_manual --raw-lidar --run-name pca_obstacle_memory_manual_1`
   - Setup: manually drive past cones/walls as they enter, leave, and re-enter lidar view.
   - Success: local costmap smearing and global obstacle persistence can be measured from PCA/lidar data.

## P2: Dynamics, Ramp, And GPS Transfer

6. `straight_distance_level_10m_1mph`
   - Commands:
     - `./run_remote.sh straight_distance_level_10m_1mph --run-name straight_distance_level_10m_1mph_forward_1`
     - `./run_remote.sh straight_distance_level_10m_1mph --run-name straight_distance_level_10m_1mph_reverse_direction_1`
   - Setup: level measured straight lane.
   - Success: physical distance, odom distance, lateral drift, and heading drift are consistent across opposite directions.

7. `in_place_yaw_ladder`
   - Command: `./run_remote.sh in_place_yaw_ladder --run-name in_place_yaw_ladder_level_1`
   - Setup: run only after `/odom`, `/local_ekf/odom`, `/encoders`, and `/tf` are confirmed present.
   - Success: odom yaw changes consistently with commanded yaw.

8. `arc_ladder`
   - Command: `./run_remote.sh arc_ladder --run-name arc_ladder_level_1`
   - Setup: large level open area.
   - Success: left/right turn radius and yaw response under translation are repeatable.

9. `ramp_ladder_full_perception`
   - Command: `./run_remote.sh ramp_ladder_full_perception --run-name ramp_ladder_full_perception_1`
   - Setup: competition-style ramp; align carefully before AUTO.
   - Success: bag captures speed loss, IMU pitch/grade, perception continuity, GPS, line detector, and costmaps on the ramp.

10. `gps_nav_observe`
    - Command: `./run_remote.sh gps_nav_observe --run-name gps_nav_observe_1`
    - Setup: RViz or GPS waypoint navigation over a small course section.
    - Success: bag captures GPS topics, goals, Nav2 status, plans, costmaps, actual path, recovery behavior, and average-speed behavior.

## Stop Conditions

- Stop P0 immediately if strict topic preflight fails; fix the missing topic before collecting more bags.
- Stop any scripted run by toggling AUTO off or hitting the wireless e-stop.
- Skip high-risk motion if the lane, ramp, or obstacle layout is not safe.
