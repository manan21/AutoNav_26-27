# AUTORESEARCH_PATH

Date: 2026-05-27
Branch: `autoresearch_path`
Base: `origin/path_following_two` at `8c6126575104501e5318ecc106871d2b2920d82b`

## Goal

Improve lidar line detection test driving performance in the ROS/Nav2 simulation so the robot reliably plans and drives through the gap between the perpendicular line and the cone to the goal point on the other side of the perpendicular line.

## Baseline

- Build command used in VM runtime workspace:
  `colcon build --symlink-install --packages-up-to autonav_detection line_layer local_mirror_layer custom_behavior_tree_plugins map_padder slam`
- Test command used in VM:
  `AUTONAV_REPO=/home/cole.guest/autonavb_runtime RUN_DIR=/tmp/autonavb_runs/<run> ./Run_LIDAR_LINE_ROS_COURSE_TEST.command`
- Baseline run: `/tmp/autonavb_runs/baseline_0/bag`
- Baseline action status: `NavigateToPose` succeeded, but measured-course analysis failed.
- Baseline trajectory crossed the perpendicular tape: `y_at_x=1.34` was `-0.042 m`; physical footprint clearance was `-0.410 m`.
- Baseline `/lidar_line_points` appeared too late, first at `8.94 s` after action start, after the robot was already past the tape. Points were on the left/top tape (`y=+0.50`) and no likely perpendicular/rightward point was detected.
- Baseline global plans stayed near `y=-0.025`, directly through the measured perpendicular tape, because no relevant hard cells existed during approach.

## Iterations

### Iteration 0: Setup

Hypothesis: Establish a reproducible baseline before making tuning or code changes.

Changes:
- Created `autoresearch_path` from latest `origin/path_following_two`.
- Added this research log.

Result:
- Build succeeded in the VM runtime workspace. The host mount is read-only inside the VM, so build/log/install were placed under `/home/cole.guest/autonavb_runtime`.
- Baseline showed the primary failure is perception feeding Nav2, not only planner inflation: the detector misses the L-shaped tape corner and publishes the perpendicular obstacle after it is too late.

### Iteration 1: Split L-shaped lidar-line clusters

Hypothesis: The perpendicular tape and left tape touch, so the lidar-line detector merges them into one L-shaped DBSCAN cluster. A single PCA line fit sees that cluster as too wide and rejects it; later, when only the left tape remains in front, it publishes a horizontal line. Decomposing wide connected clusters into multiple line-like subclusters should publish both the left tape and the perpendicular tape early enough for Nav2 to replan through the gap.

Changes:
- Updated `autonav_detection/src/lidar_line/node.cpp` to separate geometry computation from acceptance.
- Added a fallback splitter for rejected wide clusters. It repeatedly fits the principal axis, keeps narrow inliers as a line segment, removes them, and tries the remaining points. Each extracted segment must still satisfy the existing min-length, max-width, and aspect-ratio gates before publication.

Result:
- Build succeeded.
- Course test run: `/tmp/autonavb_runs/split_l_clusters_1`.
- The detector now published the perpendicular/rightward tape early: first likely perpendicular point at `7.80 s`, about `11.73 s` before physical footprint contact.
- The run did not finish before manual interrupt; `NavigateToPose` was still `EXECUTING`.
- The planner produced no `/plan` messages after the goal. The global costmap had hard cells from `y=-0.575` to `y=+0.875`, which reaches the intended pass band near the tape/cone gap.
- Trajectory still crossed the perpendicular tape (`y_at_x=1.34` was `+0.026 m`; physical footprint clearance `-0.409 m`).

Conclusion: splitting the L-shaped lidar cluster fixed the late tape perception, but the mirrored line cells plus global 1.10 m inflation overblocked the gap, so Nav2 could not find a valid route and drifted into recovery behavior.

### Iteration 2: Narrow global inflation to keep the gap open

Hypothesis: The course gap is about 1.52 m between the perpendicular tape end and the cone boundary. A 1.10 m global inflation radius around both mirrored tape cells and obstacle cells makes the gap topologically closed for Smac. Keeping local obstacle inflation conservative while reducing the global inflation radius to match the lidar-line local halo should preserve a soft stay-away gradient without erasing the only route.

Changes:
- Reduced `global_costmap.inflation_layer.inflation_radius` from `1.10` to `0.65`.

Result:
- Course test run: `/tmp/autonavb_runs/global_inflation_0p65_2`.
- Result stayed bad: the detector still saw the perpendicular line early, but Smac produced zero global plans and the robot crossed the tape during recovery/drift (`y_at_x=1.34` was `+0.024 m`, physical footprint clearance `-0.409 m`).
- Lowering the global inflation radius did not change the published lethal-cell bounds because the exact line cells are the hard obstacle; the run still entered the no-plan path.
- Offline costmap connectivity showed a footprint-feasible route exists if only exact lethal cells are treated as hard, but the start/goal footprint contains high-cost/inscribed inflated cells from stock inflation around the nearby horizontal tape. This points to line memory being mirrored before Nav2's stock inflation layer.

Conclusion: global inflation radius alone is not the right knob. The line memory layer order double-inflates tape as if it were a full obstacle, which blocks the planner before it can route through the lower gap.

### Iteration 3: Mirror line memory after stock obstacle inflation

Hypothesis: PCA/cone obstacle memory should still seed stock global inflation, but tape should use `line_layer`'s custom narrow `inscribed_radius: 0.05` and `inflation_radius: 0.65`. Moving `line_memory_mirror_layer` and `lidar_line_memory_mirror_layer` after the global `inflation_layer` should prevent robot-radius double inflation of tape while retaining persistent lethal line cells and soft line cost.

Changes:
- Reordered `global_costmap` plugins to `static_layer`, `local_mirror_layer`, `inflation_layer`, `line_memory_mirror_layer`, `lidar_line_memory_mirror_layer`.
- Restored global stock obstacle inflation radius to `1.10` so PCA/cone obstacles remain conservative; line memory now bypasses that stock inflation pass.

Result:
- Course test run: `/tmp/autonavb_runs/reordered_line_after_inflation_3`.
- The global costmap hard-cell bounds now matched the exact lidar-line costmap instead of the large stock-inflated region, confirming the plugin order change worked.
- Smac still produced zero global plans and the robot again drove straight through the tape in recovery/drift (`y_at_x=1.34` was `+0.027 m`, physical footprint clearance `-0.408 m`).
- New issue found: `/lidar_line_costmap` published almost every cell as high non-lethal cost (`94` occupancy, raw `240` after mirroring), even away from line points. Baseline bags showed the same background bug with all cells at `75`, so this predates the detector change.

Conclusion: line-layer memory ordering is correct, but the line-only costmap topic is not actually line-only. The high background cost is mirrored into global and poisons Smac's search surface.

### Iteration 4: Initialize line-layer background to free space

Hypothesis: `LineLayer::matchSize()` resizes the internal layer costmap but never forces a known free-space background before publishing `/line_costmap` and `/lidar_line_costmap`. Initializing the default value to `FREE_SPACE` and resetting after each resize should make line-only costmap topics publish `0` away from stamped line cells, so the global mirror accumulates only real tape costs.

Changes:
- Set `LineLayer`'s default cost to `FREE_SPACE` during initialization.
- Reset the internal layer map immediately after `CostmapLayer::matchSize()` before restamping persisted line points.
- Updated the line-layer header comment to match the new global ordering.

Result:
- Build succeeded.
- Course test run: `/tmp/autonavb_runs/line_background_free_4`.
- The line costmap background fix worked: `/lidar_line_costmap` no longer had an all-window high-cost background; only actual line cells and their local halo remained.
- Global planning recovered. `/plan` messages were produced, and later plans routed below the tape (`y_at_perp` eventually around `-0.746 m`).
- DWB still failed to drive the route. `/cmd_vel` angular velocity remained exactly `0.0` while the robot crept straight from the centerline to about `x=0.45`, then DWB reported `No valid trajectories out of 499`.
- Plan analysis still showed footprint overlap for many paths; the controller got an increasingly hard turn too late and could not rotate in place near the horizontal tape without footprint collision.

Conclusion: perception and global planning are now functioning, but the local controller is not starting the turn into the gap early enough. The angular sample floor and short alignment lookahead make DWB prefer straight samples until the maneuver is no longer feasible.

### Iteration 5: Let DWB follow shallow early turns

Hypothesis: The gap route starts as a gentle clockwise arc. With `min_speed_theta: 0.45` and `PathAlign.forward_point_distance: 0.10`, DWB samples either straight motion or overly sharp turns, and the sharp turns collide with the nearby horizontal tape. Lowering the angular speed floor and looking farther along the path should produce nonzero angular commands earlier, before the footprint reaches a no-turn zone.

Changes:
- Reduced `FollowPath.min_speed_theta` from `0.45` to `0.10`.
- Increased `PathAlign.forward_point_distance` and `GoalAlign.forward_point_distance` from `0.10` to `0.35`.

Result:
- Course test run: `/tmp/autonavb_runs/dwb_gentle_turns_5`.
- Result was effectively unchanged. The robot again crept straight to about `x=0.45` base-link (`current_pose` around `x=0.68`) with `y=0.0`, then DWB reported repeated `No valid trajectories out of 499`.
- `/cmd_vel_nav` and `/cmd_vel` both had max angular velocity `0.0`; the controller still never started the clockwise turn.

Conclusion: the controller tuning alone is insufficient because the global path remains too close to the tape for the rectangular footprint. DWB sees a hard maneuver too late and picks straight samples until it is boxed in.

### Iteration 6: Add footprint-scale lidar-line keep-out cost

Hypothesis: With the line-only costmap background fixed, the lidar line layer can safely use a larger inscribed/high-cost band. A `0.40 m` lidar-line `inscribed_radius` should push Smac's centerline and DWB's sampled trajectories farther below the perpendicular tape end while keeping the start pose outside the high-cost center band from the nearby horizontal tape.

Changes:
- Increased `local_costmap.lidar_line_layer.inscribed_radius` from `0.05` to `0.40`.

Result:
- Course test run: `/tmp/autonavb_runs/lidar_line_inscribed_0p40_6`.
- Failed immediately. Smac reported `Starting point in lethal space` and the behavior tree entered recovery loops near the start pose.

Conclusion: a footprint-scale lidar-line inscribed band overblocks the course because the robot starts close to the horizontal tape. The line layer needs a narrow hard/inscribed band plus a controller that can follow the curved centerline earlier.

### Iteration 7: Try Regulated Pure Pursuit for curved path tracking

Hypothesis: DWB generated no angular velocity even when Smac produced a curved path. Regulated Pure Pursuit should command curvature directly toward a lookahead point on the path, which is a better fit for the smooth lower-gap arc. Keeping `use_rotate_to_heading` off avoids in-place rotation near the horizontal tape.

Changes:
- Reverted `lidar_line_layer.inscribed_radius` to `0.05`.
- Switched `FollowPath` from DWB to `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
- Added conservative RPP params: `desired_linear_vel: 0.22`, fixed `lookahead_dist: 0.45`, collision checking enabled, cost/curvature regulation enabled, and rotate-to-heading disabled.

Result:
- Course test run: `/tmp/autonavb_runs/rpp_controller_7`.
- RPP loaded, but with collision checking enabled it immediately reported `RegulatedPurePursuitController detected collision ahead` and exceeded controller patience without driving.

Conclusion: RPP curvature tracking may still be useful, but its forward collision checker is stricter than the current global path quality and blocks all motion in this tight start configuration.

### Iteration 8: RPP curvature tracking without forward collision gate

Hypothesis: Disabling RPP's internal forward collision gate will show whether pure-pursuit curvature can actually drive the lower-gap path. This is less conservative, but local/global costmaps and the planner still shape the path; the test determines whether DWB's straight-line scoring was the main remaining controller problem.

Changes:
- Set `FollowPath.use_collision_detection: false` for the RPP controller.

Result:
- Course test run: `/tmp/autonavb_runs/rpp_no_collision_gate_8`.
- `NavigateToPose` final status: `SUCCEEDED`.
- RPP generated immediate clockwise curvature: max absolute angular velocity was `0.40 rad/s`; first nonzero commands were around `(linear=0.125, angular=-0.160)` then `(linear≈0.15, angular=-0.40)`.
- The robot moved into the lower gap instead of staying on the centerline. Sample odometry: `y=-0.113` by `x=0.399`, `y=-0.526` by `x=0.558`, and `y=-0.803` by `x=1.026`.
- Measured-course clearance passed: physical footprint min clearance to perpendicular tape was `+0.053 m`; padded footprint min clearance was `+0.023 m`. There were no footprint overlap samples.

Conclusion: the best tested configuration is the detector split plus line-layer background fix, global line memory after stock obstacle inflation, and RPP curvature tracking with its internal collision gate disabled. The path still has tight clearance, but this is the first tested configuration that actually drives through the intended lower gap and reaches the goal.

Confirmation:
- Run: `/tmp/autonavb_runs/rpp_no_collision_gate_confirm_9`.
- `NavigateToPose` final status: `SUCCEEDED`.
- RPP again produced curvature tracking with max absolute angular velocity `0.40 rad/s`.
- Measured-course clearance improved versus the first RPP run: physical footprint min clearance `+0.092 m`; padded footprint min clearance `+0.062 m`; no overlap samples.
- Sample odometry showed the same lower-gap route: `y=-0.326` by `x=0.569`, `y=-0.672` by `x=0.769`, and `y=-0.750` by `x=1.034`.

Clean-config confirmation:
- Run: `/tmp/autonavb_runs/rpp_clean_config_10` after removing unused DWB-only parameters from the active RPP block.
- `NavigateToPose` final status: `SUCCEEDED`.
- RPP max absolute angular velocity remained `0.40 rad/s`.
- Physical footprint min clearance was `+0.054 m`; padded footprint min clearance was `+0.024 m`; no overlap samples.
- Sample odometry: `y=-0.108` by `x=0.381`, `y=-0.526` by `x=0.522`, `y=-0.812` by `x=0.745`, and `y=-0.858` by `x=1.034`.
