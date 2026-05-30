# autoresearch — auto_camera Nav2 optimization (loop manual)

Entry point for the autonomous keep/discard loop. Read this, then `references/CONTEXT.md`
(runbook + the **sim-environment blocker** that must be resolved first) and `references/UNKNOWNS.md`
(idea bank). All paths are under `/home/vtcro/AutoNav_25-26`.

## Setup
1. Confirm branch `auto_camera`.
2. **Resolve the sim env** (see CONTEXT.md "BLOCKER"). The loop cannot score anything until
   `evaluate.py --course compact_baseline --runs 1 --tier 1` produces a real result with no orphaned
   `gz`/ros processes left behind.
3. Run gates: `python3 autoresearch/lib/check_footprint.py` (C-i) and
   `python3 autoresearch/lib/validate_course.py autoresearch/courses/*.yaml`.
4. Phase 0: wall-clock-calibrate one `compact_baseline` mission; auto-size tiers to the 8 h budget;
   establish the Tier-2 baseline (status=BASELINE) — the bar every KEEP must beat.

## File permissions
**EDITABLE (search space):**
- `isaac_ros-dev/src/slam/config/nav2_params_camera.yaml` — planner/MPPI/costmaps/line mirror/behaviors (YAML; no rebuild).
- `isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml` — recovery structure + PathFootprintSafe thresholds (XML; no rebuild).
- `isaac_ros-dev/src/local_mirror_layer/src/local_mirror_layer.cpp` (+hpp) — C-ii master-write/clearing (rebuild).
- `isaac_ros-dev/src/line_layer/src/line_layer.cpp` (+hpp) — local view-gated clearing (rebuild).
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/{breadcrumb_reverse,gradient_escape,...}.cpp` (rebuild).
- `isaac_ros-dev/src/autonav_detection/config/{line_detector,grade_detector}.yaml` (rebuild: data install of autonav_detection).
- detector node sources `autonav_detection/src/{line/node.cpp,grade/*.cpp}` if needed (rebuild).

**FROZEN (never edit — prevents cheating by shrinking the robot / weakening the course / gaming the scorer):**
- `autoresearch/**` (this harness, courses, worlds, references, program.md).
- `igvc_competition_sim/**` python: `course.py`, `course_monitor.py`, `mission_runner.py`,
  `generate_world.py`, `sensor_harness.py`, `camera_bridge.py`, `run_analyzer.py`, dynamics_*.
  `config/igvc_competition_compact.yaml` + `dynamics_calibration.yaml`.
- `bringup/description/shogi.urdf` (esp. `nav_center_joint`, Camera FOV/pose) + meshes.
- `scripts/analyze_*.py`, `scripts/run_lidar_line_bag_analysis.sh`.
- EKF/SLAM: `slam/config/ekf_local*.yaml`, `mapper_params_online_async.yaml` (harden nav vs drift, don't mask it).

The `robot:` block in every course YAML IS the scorer's footprint (course_monitor builds the padded
violation box from it). check_footprint.py asserts nav2/URDF/BT still match it (1e-6).

## Build rules
- YAML/BT/(launch source-path) changes -> **no rebuild**.
- C++ change -> `cd isaac_ros-dev && colcon build --symlink-install --packages-up-to <pkg> && source install/setup.bash`
  (`line_layer` | `local_mirror_layer` | `custom_behavior_tree_plugins` | `autonav_detection`).
- Pass `nav2_params:=` and `bt_xml:=` as SOURCE paths to the launch so YAML/BT edits take effect with no rebuild.

## Targets / fitness (speed-weighted, reliability-gated)
```
hard_fail(run) = mission_exit!=0 OR score.failed OR not score.finish_reached
GATE: candidate keep-eligible iff ALL N runs clean (3/3). Else DISCARD (fitness=-inf; log which run/why).
FITNESS = -(T + alpha*R + beta*J - gamma*C)
  T = mean traversal time (sim-clock; dominant)
  R = weighted recovery activations (breadcrumb, gradient, backup, spin, clearcostmap, PathFootprintSafe rejects)
  J = jitter (cmd_vel angular sign reversals + angular variance + time-below-speed)
  C = mean min course clearance (bonus; subtract)
  start: alpha~=2 s/activation, w_pfs~=0.5 s/reject, beta small, gamma~=5 per 0.1 m; calibrate in Phase 0.
KEEP iff: gate passes AND fitness strictly beats current best for that course AND no reliability metric regresses
  (violations stay 0). KEEP=commit stays; DISCARD=git reset --hard HEAD~1.
```

## Harness spec (`evaluate.py` + `lib/`) — to implement against the chosen run env
- `evaluate.py --course <id> --runs <N> --tier <1|2|3> [--run-root DIR]`: for each run call
  `lib/run_one.sh` (one headless sim + superset bag + score + group-kill + reaper), then `lib/metrics.py`
  to extract metrics, then `lib/fitness.py` to gate+score; print an ASCII report card + one machine line:
  `RESULT course=.. tier=.. runs=.. pass=.. gate=.. status=.. fitness=.. t_mean=.. viol_total=.. bc=.. ge=..
   bu=.. sp=.. cc=.. pfs=.. ang_var=.. stuck=.. min_clear=.. plan_inscribed=.. global_clear_events=.. commit=..`.
- `lib/run_one.sh`: backend-pluggable via `RUN_PREFIX` (""=host, "docker exec <ctr>", "ssh <host>").
  Replicates the proven launch+`setsid` group-cleanup from `Run_IGVC_COMPETITION_FORTRESS_TEST.command`;
  records the superset bag (CONTEXT.md); never hangs (timeouts); writes RUN_DIR/{final_score.txt,mission.log,bag}.
- `lib/reaper.sh`: kill leftover gz/ros (CONTEXT.md list), poll clear, `ros2 daemon stop`. Run before+after each run.
- `lib/metrics.py`: parse score JSON + the superset bag via `rosbag2_py` and the FROZEN `scripts/analyze_*.py`
  (see CONTEXT.md metric table). Graceful: missing topic -> metric=NA, run=INCOMPLETE; >=2 INCOMPLETE -> FLAKY.
- `lib/fitness.py`: the gate + FITNESS above; unit-self-test on synthetic metric dicts before first real use.

## The experiment loop
```
LOOP until 8h-budget - 20min:
  1. Read results/run_log.tsv (best per course, open ideas).
  2. Pick ONE change. Priority: (a) missing features [C-ii], (b) bug fixes [if baseline shows high
     pathfootprint_rejects/disruptive_aborts -> C-iii planning clearance first], (c) UNKNOWNS flips, (d) sweeps.
  3. Edit only EDITABLE files. If C++: colcon build --packages-up-to <pkg>; source install/setup.bash.
  4. git commit -m "<hypothesis>".
  5. Tier 1 (1 run): regress vs best (slower OR any violation OR more recovery) -> git reset --hard HEAD~1; log; goto 1.
  6. Tier 2 (3 runs): not 3/3 clean OR no fitness gain OR reliability regression -> reset; log; goto 1.
  7. Tier 3 (strong provisional keeps, time permitting): 5-course sweep; clean sweep -> KEEP; else reset.
  8. Append run_log row + human note in references/CONTEXT.md (hypothesis->change->result->conclusion). Prune bags. goto 1.
```

## Features to implement first (then tune)
- **C-i** done (verification gate; keep re-running it after costmap edits).
- **C-ii** camera-confirmed global line clearing: enable `line_memory_mirror_layer` `allow_decrease:true`
  + `decrease_only_in_front:true` + cone (~+/-0.96 rad, 0-5 m); widen local `line_layer` clear cone to match;
  KPI `global_clear_events>0` on sparse_lines. If 0 while `/line_costmap` clears, fix `local_mirror_layer.cpp`
  (`overwrite_master` propagation for owned cells). Mirror cone must be >= line_layer clear cone.
- **C-iii** anti-too-close-to-inflation: raise Smac `cost_penalty`; rebalance global inflation
  (radius ~0.85->~0.70 keeping the cost gradient); raise MPPI CostCritic/PathFollow weights. Success =
  pathfootprint_rejects + breadcrumb + traversal_time all down, violations 0.

## NEVER STOP / finish
Once the loop begins do not pause to ask. If out of ideas, re-read UNKNOWNS.md. At budget end: final
Tier-3 confirmation of the best config, write the summary to CONTEXT.md, commit, then `git push -u origin auto_camera`.
