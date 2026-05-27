# ROS Bag Analysis Tools

This repo keeps robot-test analysis helpers in `scripts/`. Do not rely on `/tmp` for reusable analyzers, monitors, or test helpers: `/tmp` is cleared across robot reboots and forces future agents to recreate tools from memory.

## Lidar-Line Avoidance Suite

Run the standard lidar-line bag analysis from the repo root inside the robot ROS environment:

```bash
scripts/run_lidar_line_bag_analysis.sh /path/to/bag
```

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
- `scripts/analyze_lidar_line_timeline.py`: compact event timeline for goals, commands, first line detections, measured-course footprint contact, early plans, and first DWB all-invalid sample.
- `scripts/analyze_lidar_line_plan_gap.py`: checks whether `/plan` routes through the measured tape/cone gap and reports plan clearance to lidar-line/global hard cells.
- `scripts/analyze_global_plan_costmap_collision.py`: time-aligns `/plan` and `/unsmoothed_plan` with `/global_costmap/costmap_raw` and checks whether the rectangular nav-center footprint overlaps raw lethal or inscribed global costmap cells.
- `scripts/analyze_lidar_line_course_clearance.py`: compares odometry against the measured course geometry and reports physical and padded rectangular-footprint clearance to the perpendicular tape.
- `scripts/analyze_costmap_footprint.py`: checks hard local/lidar-line costmap cells against the configured nav-center footprint over time.
- `scripts/analyze_dwb_evaluation.py`: summarizes DWB valid trajectory counts, all-invalid spans, rejection reasons, and dominant critic costs.
- `scripts/analyze_pointcloud_footprint.py`: checks point-cloud obstacle points against the footprint when point-cloud debugging is needed.

## Course Defaults

The lidar-line course geometry defaults match `docs/LIDAR_LINE_AVOIDANCE_COURSE.md`:

```bash
python3 scripts/analyze_lidar_line_plan_gap.py /path/to/bag --perp-x 1.34 --tape-right-y -0.13
python3 scripts/analyze_global_plan_costmap_collision.py /path/to/bag --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 0.50 --tape-right-y -0.13
python3 scripts/analyze_lidar_line_course_clearance.py /path/to/bag --perp-x 1.34 --perp-y-min -0.13 --perp-y-max 0.50
```

If the measured course changes, update both the course document and the defaults in `scripts/run_lidar_line_bag_analysis.sh` so future runs stay consistent.

The detector publishes completed ground-only line segments on `/lidar_line_points`, so the timeline analyzer's first perpendicular/rightward detection should appear before the measured-course footprint contact. If it appears at or after contact, the robot is probably not seeing enough floor-tape geometry early enough for Nav2 to route around it.

If the robot crosses tape after `/lidar_line_points` and `/lidar_line_costmap` saw it, use `analyze_global_plan_costmap_collision.py` first. If `/unsmoothed_plan` is safe but `/plan` is not, the smoother is clipping the path. If both plans overlap raw global lethal or inscribed cells, the planner/costmap representation is still unsafe. If raw global cells are missing, debug line memory mirroring or costmap timing before tuning DWB.

## Adding New Helpers

For scratch experiments, `/tmp` is fine. Once a helper is used to make a tuning decision or appears in a test report, move it into `scripts/`, give it a short docstring and `--help`, and add it to this document. If it is part of the standard lidar-line pass/fail workflow, also add it to `scripts/run_lidar_line_bag_analysis.sh`.
