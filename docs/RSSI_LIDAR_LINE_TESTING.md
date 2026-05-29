# RSSI LiDAR Line Testing

Use this workflow when the course uses plain white tape that does not appear as
SICK reflector returns. The goal is to spend course access on one raw data
capture, then tune RSSI thresholds offline.

## Branch And Build

Use branch `rssi_lines`, built from `autoresearch_path_nav_fix`.

```bash
cd /autonav/isaac_ros-dev
colcon build --symlink-install --packages-up-to autonav_detection
source install/setup.bash
```

## Short Course Capture

Launch the normal stack pieces needed for LiDAR and TF, then record one short
bag:

```bash
./config/record_lidar_rssi_lines.sh
```

Default duration is 90 seconds. Override with:

```bash
RSSI_LINE_RECORD_SECONDS=60 ./config/record_lidar_rssi_lines.sh course_lines_1
```

During the recording:

1. Hold still with tape visible for 10-15 seconds.
2. Slowly move past or along the tape.
3. End with nearby non-line ground visible for a false-positive baseline.

Do not tune live on the course. The bag includes `/cloud_all_fields_fullframe`
so thresholds can be replayed offline.

## Offline RSSI Sweep

Run the sweep analyzer from the repo root in the ROS environment:

```bash
python3 scripts/analyze_lidar_rssi_sweep.py /autonav/logs/rssi_lines/course_lines_1/bag \
  --positive-window 0 20 \
  --negative-window 70 90 \
  --csv /autonav/logs/rssi_lines/course_lines_1/rssi_sweep.csv \
  --clusters-csv /autonav/logs/rssi_lines/course_lines_1/rssi_clusters.csv
```

The analyzer ranks combinations of:

- `min_intensity`
- `adaptive_range_bin_m`
- `adaptive_min_delta`
- `adaptive_stddev_multiplier`

It reports accepted tape-like clusters, candidate density, cluster length, and
cluster width. Prefer settings that create stable long clusters on tape while
keeping candidate density low enough to avoid ground floods.

The `--positive-window` interval should cover the part of the recording where
plain white tape is clearly visible. The `--negative-window` interval should
cover nearby ground without tape. Adjust those times if the capture is shorter
or the choreography changes.

The default sweep is subsampled for Jetson-side turnaround:
`--cloud-stride 10 --point-stride 4`. If the first result is ambiguous, rerun a
smaller threshold range with denser sampling, for example `--cloud-stride 3
--point-stride 1`.

For timing-window analysis, make sure the sampled time range reaches the
negative window. If the analyzer warns that a window is not covered, rerun with
`--max-clouds 0` or a larger `--max-clouds`; otherwise the ranking can make a
false-positive interval look clean.

## Live RSSI Detector Test

After choosing values from the bag sweep, run the RSSI profile:

```bash
./config/run-lidar-rssi-lines.sh \
  -p min_intensity:=40000.0 \
  -p adaptive_range_bin_m:=0.15 \
  -p adaptive_min_delta:=4000.0 \
  -p adaptive_stddev_multiplier:=2.0 \
  -p cluster_min_length_m:=0.70 \
  -p cluster_max_width_m:=0.08
```

Then inspect in RViz:

- `/lidar_line_detection/debug/points`
- `/lidar_line_points`
- `/lidar_line_costmap`
- `/lidar_line_detection/diagnostics`

Keep the default `lidar_line_detector.yaml` reflector profile unchanged for
retroreflective tape tests.

## 2026-05-29 Course Bag Result

Bag: `course_lines_20260529_132420`.

The existing reflector detector produced 548 `/lidar_line_points` messages and
all were empty, confirming that the plain white course tape is not usable as a
SICK reflector target.

RSSI can see the tape, but RSSI-only detection is not clean enough to trust as a
hard competition line source from this bag alone. The best replay-tested
tradeoff was:

```bash
-p min_intensity:=40000.0 \
-p adaptive_range_bin_m:=0.15 \
-p adaptive_min_delta:=4000.0 \
-p adaptive_stddev_multiplier:=2.0 \
-p cluster_min_length_m:=0.70 \
-p cluster_max_width_m:=0.08
```

The full-density analyzer showed useful detections in both line-present windows
and zero detections in the sampled empty-lot window, but C++ bag replay still
produced empty-lot line outputs. Tightening geometry or intensity enough to
remove those replay false positives also removed most or all tape detections.
Use this RSSI profile only for live RViz inspection or as a secondary signal
until it is gated by another detector or validated on more course backgrounds.
