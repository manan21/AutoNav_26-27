# auto_camera build-loop — context, runbook, and research log

Branch: `auto_camera` (off `autoresearch_path_nav_fix`). Started 2026-05-30.
Plan: `/home/vtcro/.claude/plans/we-need-to-improve-purring-journal.md`.

## Goal
Stop the robot getting stuck / jittering / overusing breadcrumb-reverse recovery; traverse the IGVC
course faster and reliably; meet qualification. Optimize the **camera profile**
`isaac_ros-dev/src/slam/config/nav2_params_camera.yaml` (SmacPlannerLattice + MPPI + `bt_nav.xml`)
via a Karpathy keep/discard loop scored in the Gazebo Fortress sim.

## User decisions (locked)
- Objective = **speed-weighted, reliability-gated**: any run that crosses a line / hits an obstacle /
  fails to complete is discarded; among clean runs minimize traversal time, minor penalties for
  recovery activations + jitter.
- Reliability gate = **3/3** repeat runs clean before "keep"; metrics averaged.
- Course suite = **5 themed scenarios** (built + validated, see below).
- **Tiered eval**: Tier 1 = 1 short course ×1 (screen, discard on regression); Tier 2 = same ×3 (gate);
  Tier 3 = full 5-course sweep (clean sweep required to keep).

## !!! BLOCKER: the sim cannot run on this host (must resolve before the loop runs) !!!
This x86 laptop is **not** a runnable sim environment. Verified 2026-05-30:
- Dev env is a **Jetson (aarch64)** docker image chain `dev:koopa-kingdom` <- `autonav:koopa-kingdom`
  <- `nvcr.io/nvidia/l4t-jetpack:r36.4.0` (`env/docker/dockerfiles/`). None of these images exist here,
  and the L4T base is wrong-arch for x86.
- Host `/opt/ros/humble` is partial: **`ros_gz_bridge`/`parameter_bridge` and `pointcloud_to_laserscan`
  are missing** (both required by `igvc_competition.launch.py`). Host `gz` CLI is broken.
- Submodules `pointcloud_to_laserscan` and `zed-ros2-wrapper` are **not checked out**.
- `install/` is stale: `igvc_competition_sim`, `gps_waypoint_handler`, `map_padder` not built; `slam`
  `config/` not installed.
- **No passwordless sudo** -> cannot apt-install ros-gz here.

What DOES work on the host (verified): the pure-python world generator, the course loader, `rosbag2_py`
deserialization, numpy/scipy. So course authoring/validation and bag analysis run here; only the live
sim is blocked.

### To unblock — pick one:
- **(A) Point to a prepared machine** (the prior autoresearch ran on a VM `/home/cole.guest/autonavb_runtime`).
  The harness `run_one.sh` will be written with a pluggable backend: `RUN_PREFIX="ssh <host>"` or
  `RUN_PREFIX="docker exec <ctr>"` so it runs wherever the stack lives.
- **(B) Provision this host** (needs sudo): `apt install ros-humble-ros-gz ros-humble-pointcloud-to-laserscan`
  (+ Gazebo Fortress `ignition-fortress`), `git submodule update --init isaac_ros-dev/src/pointcloud_to_laserscan`,
  then `cd isaac_ros-dev && colcon build --symlink-install`. (Run interactive sudo via `! <cmd>` in the session.)

## Verified run mechanism (for the harness, once unblocked)
- Launch: `ros2 launch igvc_competition_sim igvc_competition.launch.py course_config:=<yaml> world:=<sdf>
  line_detection_mode:=camera nav2_params:=<SRC>/slam/config/nav2_params_camera.yaml
  bt_xml:=<SRC>/slam/behavior_trees/bt_nav.xml gazebo_server_only:=true`.
  - **Pass `world:=` explicitly** — the launch's `world` arg defaults to the compact SDF and is NOT
    derived from `course_config` (the .command never overrode it because it only ran compact).
  - **Pass `nav2_params:=` and `bt_xml:=` as SOURCE paths** — the install copies are stale/missing, and
    source paths mean YAML/BT edits need NO rebuild. (`igvc_competition_sim` package share is NOT
    installed, so the package must be built once anyway for the launch/nodes to resolve.)
- Mission: `timeout <T> ros2 run igvc_competition_sim igvc_mission_runner --course-config <yaml>
  --timeout-sec <T>` — sends each `mission_waypoints` entry as a NavigateToWaypoint goal; exit 0 iff ALL
  succeed; per-waypoint log `"[label] status=N succeeded=BOOL final_distance=.. reason=.."`.
- Score (authoritative gate): `ros2 topic echo --once /igvc_sim/score` -> JSON
  `{course_id, failed, failures[], distance_m, max_speed_mps, finish_reached, speed_check_complete}`.
  `course_monitor.py` judges the **padded footprint (+/-0.595 x +/-0.460 m at nav_center +0.225)** on
  `/odom`. failure strings: `tape_crossing:<n>`, `obstacle_contact:<n>`, `pothole_contact:<n>`,
  `ramp_edge_departure:<n>`, `max_speed_exceeded`, `first_44ft_speed_below_1mph`, `blocking_stop_over_60s`.
- Cleanup: launch under `setsid`; INT/TERM/KILL the process GROUP; then a reaper `pkill`s
  `gz sim|ruby|parameter_bridge|controller_server|planner_server|bt_navigator|behavior_server|costmap|
  autonav_detection|component_container`; `ros2 daemon stop`; unique `ROS_DOMAIN_ID`. **One sim at a
  time** (RTX 3050 Ti has 4 GB VRAM; CUDA line detector + Gazebo render compete; watch CUDA OOM).
- **Recovery signals are NOT in the stock test bag** — the harness records its OWN superset bag adding
  `/rosout`, `/back_up/_action/status`, `/drive_on_heading/_action/status`, `/spin/_action/status`,
  `/unsmoothed_plan` (and may drop heavy image/cloud topics to save disk).
- ZED real HFOV = **1.918862 rad (~110 deg)**, pitched down 0.349 rad; usable ground range ~5 m
  (`line_detector.yaml` max_depth/base_max_x). Use this for the C-ii clearing cone (~ +/-0.96 rad, 0-5 m).

## Status (2026-05-30)
DONE + VERIFIED on host:
- Branch `auto_camera` created.
- **C-i footprint accuracy: PASS** — consistent across course RobotSpec, nav2 local+global, URDF
  nav_center_joint (0.225), BT PathFootprintSafe. Padded scorer box +/-0.595 x +/-0.460 at nav_center.
  Re-run anytime: `python3 autoresearch/lib/check_footprint.py`.
- **5 courses authored + worlds generated + feasibility-validated** (padded-robot connectivity, all
  waypoints+finish reachable): compact_baseline, tight_gaps, dense_obstacles, sparse_lines, ramp_turns.
  Frozen `robot:` block byte-identical across all 5. Re-run: `python3 autoresearch/lib/validate_course.py
  autoresearch/courses/*.yaml`. (Necessary, not sufficient — route feasibility confirmed on first sim baseline.)

DEFERRED until the sim env is available (so they are built+tested for real, not blind):
- Executable harness `evaluate.py` / `lib/{run_one.sh,reaper.sh,metrics.py,fitness.py}` — fully specified
  in `program.md`. The run-backend is pluggable per the unblock choice above.
- Phase-0 wall-clock calibration + tier auto-sizing; baseline; the C-ii / C-iii experiments; the loop.

## Iteration log
(_baseline + each kept/discarded experiment recorded here in the AUTORESEARCH_PATH.md style once the loop runs._)
- 2026-05-30 setup: branch + scaffold + 5 validated courses + footprint check. Loop blocked on sim env.
