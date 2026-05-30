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

- **Executable harness built + scoring path VALIDATED** (user chose "build now, validate later"):
  - `lib/metrics.py` (self-contained rosbag2 parser: clock-mapped traversal time, recovery counts from
    exact /rosout strings + behavior action statuses, cmd_vel jitter/stuck, course-geometry clearance,
    score JSON + mission.log) and `lib/fitness.py` (3/3 reliability gate + speed-weighted fitness +
    report card / RESULT line) are **unit-tested** by `lib/selftest_metrics.py` against a synthetic
    rosbag2 (16/16 asserts pass). `evaluate.py --score-existing` end-to-end-tested (gate FAIL on a mixed
    candidate, correct run_log row). Re-run: `python3 lib/selftest_metrics.py`.
  - `lib/run_one.sh` (headless launch + superset bag + setsid group-cleanup + reaper, pluggable paths)
    and `lib/reaper.sh`: syntax-checked + arg/file-validation tested; **the live sim-run path is
    UNVALIDATED** (no runnable sim here).

- **C-ii implemented (config-only) in nav2_params_camera.yaml** -- UNVALIDATED in sim:
  `line_memory_mirror_layer` now `allow_decrease:true` + `decrease_only_in_front:true` + cone
  +/-0.96 rad / 0-5 m; local `line_layer` clear cone widened to +/-0.85 rad / 0.8-5 m.
  **Verified by reading `local_mirror_layer.cpp`**: keeping `overwrite_master:false` is correct and
  safer -- the master is rebuilt each cycle, so clearing this layer's own accumulated cell (via the
  allow_decrease path, lines 525-528) is sufficient to free a line-ONLY master cell, while the
  master-write max-merge (lines 592-595) still prevents a line halo from erasing a real PCA obstacle at
  a shared cell. So NO C++ change was needed (the Plan's "budget a C++ iteration" precaution is resolved).
  KPI to confirm on first sim run: `global_clear_events > 0` on sparse_lines AND no PCA obstacle erosion.

- **C-iii implemented in nav2_params_camera.yaml** -- UNVALIDATED in sim: Smac `cost_penalty` 2.0 -> 3.0
  (route the centerline farther from inflation by construction -> fewer PathFootprintSafe rejects /
  breadcrumb fallbacks). Soft preference only; lethal topology unchanged so the 5 ft passage stays usable.

DEFERRED until the sim env is available:
- Phase-0 wall-clock calibration + tier auto-sizing; the baseline; running the keep/discard loop;
  validating C-ii (`global_clear_events`) and C-iii (`pathfootprint_rejects`/`breadcrumb`/time down,
  0 violations); costmap-based metrics (executed_lethal_clear / plan_inscribed_clear / global_clear_events)
  wired from the FROZEN scripts/analyze_*.py.

## How to run once a sim is available
1. Get a runnable sim (see BLOCKER). Build the ws once: `cd isaac_ros-dev && colcon build --symlink-install`.
2. Gates: `python3 .../autoresearch/lib/check_footprint.py` and `... validate_course.py courses/*.yaml`.
3. Smoke: `python3 .../autoresearch/evaluate.py --course compact_baseline --runs 1 --tier 1`
   (confirm a RESULT line + no orphaned gz/ros: `pgrep -fa 'gz sim|controller_server'` empty).
4. Baseline (Tier-2 x3 on compact_baseline), then follow `program.md`'s loop. C-ii/C-iii are already the
   working-tree's first changes -- baseline them, then keep/discard per the gate.

## Iteration log
(_baseline + each kept/discarded experiment recorded here in the AUTORESEARCH_PATH.md style once the loop runs._)
- 2026-05-30 setup: branch + scaffold + 5 validated courses + footprint check (PASS).
- 2026-05-30 harness: metrics.py + fitness.py built and self-tested (16/16); evaluate.py/run_one.sh/reaper.sh
  built (sim-run path unvalidated). C-ii (config-only line clearing) + C-iii (cost_penalty 3.0) implemented,
  UNVALIDATED. Loop run still blocked on a runnable sim env.
- 2026-05-30 SIM UNBLOCKED: user installed Gazebo Fortress 6.16 + ros_gz (sudo). Brought the sim up on this
  x86 host (commit b7278e96): prefer `ign gazebo` over Classic `gz sim`; add gz Sensors-system plugin so the
  camera renders headless (GLX on :1) -> /line_points ~1500/run; tolerate missing rclpy RCLError in 3 sim
  nodes; fix run_one.sh REPO depth; build copy-mode (setuptools 80 breaks --symlink-install). Harness now
  VALIDATED end-to-end against the real sim (camera line + PCA obstacle + Nav2 + scorer + metrics).
- 2026-05-30 BASELINE (compact_baseline, current C-ii/C-iii config): FAILS. The robot traverses most of the
  course but (a) gets STUCK >60s mid-course -> timeout/non-completion (blocking_stop_over_60s), (b) clips
  right_boundary_1 + pothole_0 early (min_course_clear ~ -0.16 to -0.45), (c) slow (~0.40-0.42 m/s over the
  first 44ft, under the 0.447 = 1 mph minimum). High run-to-run variance (one run finished at ~144s, another
  stalled) -> the 3-run gate is essential. While failing, rank candidates by PROGRESS (distance reached,
  fewer violations, better clearance, completion) since fitness is gate-gated. Primary targets: eliminate the
  stuck-stall, raise speed, keep margin off boundaries/potholes.
- 2026-05-30 exp1 (in progress): vx_std 0.25->0.40 (the config comment notes 0.25 made MPPI dawdle at
  ~0.2 m/s; raise it to use the 0.5 cap -> faster, more decisive, less stall).
