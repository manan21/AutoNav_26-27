# Gazebo Canonicalization Work Log

Keep this file current. It is the durable handoff for future agents.

## Status

- Branch: `auto_camera`
- Current head when this log was created: `6469ed59`
- Non-GPU phase: offline bag/video analysis implemented and run; sim runtime
  validation remains pending
- GPU phase: deferred until an NVIDIA Linux environment is available

## Environment Notes

- `autonav-gazebo-sim` has ROS 2 Humble and `rosbag2_py`, but currently only
  mounts `/Users/cole/code/git/AutoNav_25-26-gazebo`.
- `autonav-ros22` mounts `/Users/cole`, `/Users/cole/code/git`, the selected
  bags, and `AutoNavB`, but does not have `rosbag2_py`.
- Installed `rosbags` into the `autonav-ros22` user environment so selected
  bags can be decoded without copying multi-GB data into the Gazebo VM.

## Data Inventory

Selected dynamics bags:

- `/Users/cole/autonav_bags/jetson_practice_course_20260529_211047/arc_ladder_1`
- `/Users/cole/autonav_bags/jetson_practice_course_20260529_211047/in_place_yaw_ladder_1`
- `/Users/cole/autonav_bags/jetson_practice_course_20260529_211047/straight_distance_drift_low_2`

Selected perception bags:

- `/Users/cole/autonav_bag_backups/manual_course_rerun_5`
- `/Users/cole/autonav_bag_backups/manual_course_rerun_7`
- `/Users/cole/autonav_bag_backups/camera_line_detection_rerun`
- `/Users/cole/autonav_bag_backups/camera_line_detection_rerun_2feet_stationary`

Selected videos:

- `/Users/cole/Downloads/arc_ladder_1.MOV`
- `/Users/cole/Downloads/in_place_yaw_ladder_1.MOV`
- `/Users/cole/Downloads/straight_distance_drift_low_2.MOV`
- `/Users/cole/Downloads/manual_course_rerun_5.MOV`
- `/Users/cole/Downloads/manual_course_rerun_7.MOV`
- `/Users/cole/Downloads/camera_line_detection_rerun.MOV`
- `/Users/cole/Downloads/camera_line_detection_rerun_two_feet_stationary.MOV`

## Validated

- `auto_camera` already contains the real-robot bag persistence fixes from
  `autoresearch_path_nav_fix`.
- Footprint consistency gate passed after the merge:
  `autoresearch/lib/check_footprint.py`.
- The selected rerun bags and videos are present locally, but the four rerun
  bags are under `/Users/cole/autonav_bag_backups`, not the older
  `jetson_practice_course_20260529_211047` root.
- `scripts/analyze_gazebo_canon_offline.py` compiles locally and ran
  successfully inside `autonav-ros22` using the pure-Python `rosbags` reader.
- Generated durable outputs:
  - `docs/GAZEBO_CANON_OFFLINE_REPORT.md`
  - `docs/gazebo_canon_offline_report.json`
- Field-test roadmap added in `docs/REAL_ROBOT_FIELD_TEST_ROADMAP.md`.
- Real-robot calibration suite now includes strict `canon_full` topic capture
  plus named profiles for camera-line projection, Nav2 gap debug, costmap
  memory, PCA obstacle memory, level-ground straight distance, ramp full
  perception, and GPS/Nav2 observation.

## Findings

- Camera line detection is currently the highest-risk transfer issue.
  Three of four camera/perception bags had non-empty
  `/line_detection/line_pixels` but zero `/line_points`:
  `manual_course_rerun_5`, `camera_line_detection_rerun`, and
  `camera_line_detection_rerun_2feet_stationary`.
- In those three runs, `/line_costmap` stayed empty even though pixels were
  detected. A robot stack using that costmap could drive over visible tape.
- The only selected run with non-empty projected camera line points was
  `manual_course_rerun_7`, with max `/line_points` count 86 and
  `/line_costmap` max nonzero cells 5522.
- Dominant camera-line diagnostic reason across the selected perception bags:
  `stamped TF unavailable` (1228 messages). The next major issue in the
  working run was RGB/depth desynchronization.
- Only `manual_course_rerun_7` contains raw ZED RGB+depth image topics in the
  selected perception set. The other three perception bags still contain line
  detector diagnostics/debug products, but not raw ZED image/depth streams.
- TF summary shows the failing stationary/rerun bags were missing enough
  stamped dynamic TF for the camera projection path. The worst case,
  `camera_line_detection_rerun_2feet_stationary`, had only wheel-link dynamic
  transforms on `/tf`; no `/odom` or `/local_ekf/odom` topic was present in
  that bag.
- The selected manual/calibration bags contain zero Nav2 plan messages. They
  validate perception/costmap/dynamics symptoms, but cannot validate whether
  the global planner planned through a line, obstacle, or false gap. Use a
  real autonomous nav debug bag or a live sim autonomy run for plan/costmap
  collision clearance analysis.
- `straight_distance_drift_low_2` measured physical forward travel 9.3726 m
  and right drift 1.5288 m. `/odom` reported 7.9542 m forward, giving a
  physical/odom forward ratio of 1.178. Keep the right drift diagnostic only
  because the parking lot slope likely contaminated it.
- `in_place_yaw_ladder_1` commanded yaw up to 1.0 rad/s, but odom yaw changed
  only 0.022 degrees. Do not tune Gazebo yaw dynamics from this bag until the
  odom/encoder yaw path is explained.
- Updated `igvc_competition_sim/config/dynamics_calibration.yaml` to mark
  `in_place_yaw_ladder_1` as excluded from default yaw tuning and to record
  the latest straight-run odom diagnostic values:
  `/odom` forward 7.9542 m, `/local_ekf/odom` forward 8.2193 m, physical
  forward 9.3726 m.

## Attempted But Rejected / Do Not Repeat

- Do not use `autonav-gazebo-sim` directly for these local bags until its Lima
  mounts are changed; it cannot currently see `/Users/cole/autonav_bags`,
  `/Users/cole/autonav_bag_backups`, or `AutoNavB`.
- Do not copy the multi-GB bags into the Gazebo VM just to decode them. Use
  `autonav-ros22` plus `rosbags`, or fix the VM mount configuration later.
- Do not tune `flat_ground_yaw_bias_radps` from
  `straight_distance_drift_low_2` right drift alone; the parking lot was sloped.

## Remaining Work

- Fix or validate the camera line projection path before trusting sim-based
  line avoidance. Real bags show segmentation pixels without 3-D line points
  when TF/depth sync fails.
- Collect the P0/P1/P2 field-test roadmap, starting with
  `camera_line_static_canon`, `camera_line_motion_creep`, and
  `nav_debug_driveway_gap`.
- Add an autonomous nav debug dataset with `/plan`, `/unsmoothed_plan`,
  `/transformed_global_plan`, local/global costmaps, line costmap, obstacle
  sources, TF, odom, and all line detector diagnostics. The selected manual
  bags are not enough for planner clearance analysis.
- Explain the `in_place_yaw_ladder_1` odom result before using yaw ladder data
  for physics calibration.
- Treat straight-line odom scale as a candidate diagnostic; do not encode the
  sloped-lot lateral drift as a flat-world sim bias.
- Run Gazebo on an NVIDIA Linux environment and compare simulated camera
  images, white tape segmentation, projected line points, line costmap
  persistence, obstacle costmap persistence, plans, controller speed, recovery
  behavior, and ramp traversal against the report values.
- Defer CUDA/Gazebo camera-render validation to an NVIDIA environment.
