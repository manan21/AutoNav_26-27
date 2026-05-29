# ROS Bag Analysis Tools

This repo keeps robot-test analysis helpers in `scripts/`. Do not rely on `/tmp` for reusable analyzers, monitors, or test helpers: `/tmp` is cleared across robot reboots and forces future agents to recreate tools from memory.

## Lidar-Line Avoidance Suite

Run the standard lidar-line bag analysis from the repo root inside the robot ROS environment:

```bash
scripts/run_lidar_line_bag_analysis.sh /path/to/bag
```

For ROS simulation scenario bags, pass the scenario config so the analysis can
use scenario-specific tape, cone, and station metadata:

```bash
scripts/run_lidar_line_bag_analysis.sh /path/to/bag \
  --scenario-config /path/to/lidar_line_sim/config/scenarios/canonical_5ft_gap.yaml
```

Add `--strict-scenario-geometry` only after the scenario has an accepted
baseline; without it, station bands and padded footprint overlap are
diagnostics while physical tape/cone overlap remains a hard failure.

The lidar-line test runner calls this suite automatically after each recorded test unless `LIDAR_LINE_TEST_SKIP_ANALYSIS=1` is set:

```bash
isaac_ros-dev/config/run_lidar_line_test.sh
```

The runner writes the combined output to:

```text
/autonav/logs/<run_name>/analysis.log
```

## Included Analyzers

- `scripts/analyze_lidar_line_bag.py`: high-level test summary, nav-center travel, commands, odometry, line detections, costmap persistence, action statuses, and PCA point-cloud presence.
- `scripts/analyze_bt_control_churn.py`: classifies Nav2 BT/controller churn. It separates normal `FollowPath` replacement caused by fresh plans from disruptive aborts, reports `ComputePathToPose` abort bursts, checks final `/cmd_vel` gaps, and summarizes `/rosout` safety/recovery events when `/rosout` is recorded.
- `scripts/analyze_lidar_line_timeline.py`: compact event timeline for goals, commands, first line detections, measured-course footprint contact, early plans, and first DWB all-invalid sample when `/evaluation` exists.
- `scripts/analyze_lidar_line_plan_gap.py`: checks whether `/plan` routes through the measured tape/cone gap and reports plan clearance to lidar-line/global hard cells.
- `scripts/analyze_lidar_line_scenario.py`: metadata-driven simulator scenario analyzer. It checks executed physical and padded footprint clearance against configured tape and cone geometry, then reports scenario station bands such as centering or legal-route selection. Standard runs fail only physical overlap; strict scenario geometry also fails padded overlap and station-band misses.
- `scripts/analyze_global_plan_costmap_collision.py`: time-aligns `/plan` and `/unsmoothed_plan` with `/global_costmap/costmap_raw` and checks whether the rectangular nav-center footprint overlaps raw lethal global costmap cells. It still reports inscribed-cell clearance, but the standard suite treats inscribed overlap as a diagnostic because global inflation already encodes tape/obstacle clearance.
- `scripts/analyze_lidar_line_course_clearance.py`: compares odometry against the measured course geometry and reports physical and padded rectangular-footprint clearance to the perpendicular tape. In the standard suite any physical or padded overlap fails the analysis.
- `scripts/analyze_costmap_footprint.py`: checks hard local/lidar-line costmap cells against the configured nav-center footprint over time.
- `scripts/analyze_dwb_evaluation.py`: optional legacy/controller-debug analyzer for DWB `/evaluation`; the active Nav2 profiles use MPPI, so missing `/evaluation` is expected unless DWB is restored for a comparison run.
- `scripts/analyze_pointcloud_footprint.py`: checks point-cloud obstacle points against the footprint when point-cloud debugging is needed.

## Course Defaults

The lidar-line course geometry defaults match `docs/LIDAR_LINE_AVOIDANCE_COURSE.md`:

```bash
python3 scripts/analyze_lidar_line_plan_gap.py /path/to/bag --perp-x 1.34 --tape-right-y -0.13
python3 scripts/analyze_global_plan_costmap_collision.py /path/to/bag --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 1.524 --tape-right-y -0.13 --half-length 0.595 --half-width 0.46
python3 scripts/analyze_lidar_line_course_clearance.py /path/to/bag --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 1.524 --padding 0.05 --fail-on-overlap
```

If the measured physical course changes, update both the course document and the canonical defaults in `scripts/run_lidar_line_bag_analysis.sh` so future runs stay consistent. For simulator scenarios, prefer adding or updating the scenario YAML and passing `--scenario-config` instead of hard-coding another analyzer invocation.

The DWB evaluation analyzer is optional in the standard suite because the active Nav2 profiles use MPPI. Missing `/evaluation` should not mask the hard acceptance gates: executed lethal footprint overlap or measured tape overlap still exits nonzero. For conservative experiments, add `--fail-on-overlap` or `--fail-on-inscribed-overlap` to the global plan analyzer.

The detector publishes completed ground-only line segments on `/lidar_line_points`, so the timeline analyzer's first perpendicular/rightward detection should appear before the measured-course footprint contact. If it appears at or after contact, the robot is probably not seeing enough floor-tape geometry early enough for Nav2 to route around it.

Do not treat raw `FollowPath` ABORTED counts as a hard failure by themselves. With frequent replanning, Nav2 can terminate an old `FollowPath` action when a fresh safe path replaces it. Use `analyze_bt_control_churn.py`: disruptive `FollowPath` aborts, `ComputePathToPose` abort bursts, final `/cmd_vel` gaps, recovery waits, and safety-gate rejects are the meaningful signals.

If the robot crosses tape after `/lidar_line_points` and `/lidar_line_costmap` saw it, use `analyze_global_plan_costmap_collision.py` first. If `/unsmoothed_plan` is safe but `/plan` is not, the smoother is clipping the path. If either plan overlaps raw global lethal cells, the planner/costmap representation is still unsafe. If raw global cells are missing, debug line memory mirroring or costmap timing before tuning the controller.

## Adding New Helpers

For scratch experiments, `/tmp` is fine. Once a helper is used to make a tuning decision or appears in a test report, move it into `scripts/`, give it a short docstring and `--help`, and add it to this document. If it is part of the standard lidar-line pass/fail workflow, also add it to `scripts/run_lidar_line_bag_analysis.sh`.
