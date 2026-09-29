# autonav_viz

RViz visualization suite for the CV pipeline (IGVC Self-Drive). It shows how
the vehicle perceives:

1. **Lanes:** white outer lanes vs yellow center markings (masks + 3D points)
2. **Obstacles:** classified barrels / potholes / pedestrians / signs (3D boxes + labels), with the raw lidar evidence underneath
3. **Signals:** live traffic-light color state and sign classification readouts
4. **BEV / mapping:** local costmap, camera line costmap, planned path, and a bird's-eye camera warp

This package does **not** do perception. It owns the RViz config and a **topic
contract**. Displays for outputs that don't exist yet are fed by
`perception_viz_stub` until the real detectors publish on the same topics.
After that, the displays work with no RViz changes.

## Run

```bash
# Laptop (native RViz, no build needed): uses cv_pipeline.rviz by default
./isaac_ros-dev/config/run-rviz.sh

# Or as a package (after colcon build)
ros2 launch autonav_viz viz.launch.py
ros2 launch autonav_viz viz.launch.py use_stub:=true                        # placeholder perception
ros2 launch autonav_viz viz.launch.py use_stub:=true stub_classes:=barrel,traffic_light
ros2 launch autonav_viz viz.launch.py use_sim_time:=true                    # rosbag replay (--clock)
```

**Views:** the default is top-down (BEV) and tracks `base_link` in the `odom`
fixed frame. Switch to the saved *Chase Cam (3D)* view in the Views panel for a
perspective shot of the boxes and labels.

## Topic contract

| Display group | Topic | Type | Publisher today |
|---|---|---|---|
| Lanes | `/line_detection/debug/overlay` | `sensor_msgs/Image` (bgr8) | `line_detector` (real, 2 Hz) |
| Lanes | `/line_detection/debug/mask` | `Image` (mono8), white lines | `line_detector` (real) |
| Lanes | `/perception/lanes/yellow_mask` | `Image` (mono8), yellow center | **stub** |
| Lanes | `/perception/lanes/mask_color` | `Image` (bgr8): white px = white lane, yellow px = yellow lane | **stub** (composites real white mask + yellow) |
| Lanes | `/lines_pointcloud` | `PointCloud2` (`odom`) | `line_detector` (real) |
| Lanes | `/lidar_line_detection/debug/points` | `PointCloud2` | `lidar_line_detector` (real) |
| Obstacles | `/perception/objects/markers` | `visualization_msgs/MarkerArray` | **stub** |
| Obstacles | `/scan_pca_filtered_points`, `/scan_pca_filtered` | `PointCloud2` / `LaserScan` | `grade_detector` (real) |
| Signals | `/perception/signals/markers` | `MarkerArray` | **stub** |
| Signals | `/perception/detections/image` | `Image` (bgr8), 2D boxes on camera | **stub** |
| BEV | `/local_costmap/costmap`, `/line_costmap`, `/lidar_line_costmap`, `/global_costmap/costmap`, `/terrain/grade_map` | `nav_msgs/OccupancyGrid` | Nav2 / `line_layer` / `grade_detector` (real) |
| BEV | `/plan` | `nav_msgs/Path` | Nav2 planner (real) |
| BEV | `/transformed_global_plan`, `/trajectories` | `Path` / `MarkerArray` | MPPI, only with `FollowPath.visualize: true` (off by default) |
| BEV | `/perception/bev/image` | `Image` (bgr8), top-down warp | **stub** |

### Marker conventions (`/perception/objects/markers`)

- One marker per object, with `ns` = class name and a stable `id` per track.
- A second `TEXT_VIEW_FACING` marker in `ns` = `<class>_label` above it, with text `"<class> <conf>"`.
- `header.frame_id`: `base_link` or `odom`. Set `lifetime` ≈ 1–2 publish periods so stale detections disappear.

| Class | Shape | Color (RGBA) |
|---|---|---|
| `barrel` | CYLINDER | orange (1, 0.5, 0) |
| `pothole` | flat CYLINDER | white |
| `pedestrian` | CUBE | blue (0.2, 0.4, 1) |
| `stop_sign` | CUBE | red |
| `road_sign` | CUBE | yellow |
| `traffic_light` | CUBE | gray; label includes `[RED/YELLOW/GREEN]` |

`/perception/signals/markers` holds the driver-facing readout:

- `ns=signal_readout` TEXT markers, e.g. `"TRAFFIC LIGHT: GREEN (0.93)"` colored by state, and `"SIGN: STOP (0.88)"`
- `ns=light_state` SPHEREs colored by the current light state

The authoritative palette lives in `CLASS_STYLE` / `LIGHT_STATES` in
`autonav_viz/perception_viz_stub.py`.

## perception_viz_stub (testing only)

The stub publishes placeholder data on every **stub** topic above; images are
watermarked `STUB`. It uses a pinhole model of the ZED mount from `shogi.urdf`
(x 0.6576, z 0.3072, pitch 0.349 rad, ground at z -0.113 in `base_link`). With
this model:

- Synthetic road, projected 2D boxes and BEV warp all agree geometrically.
- When `/line_detection/debug/raw` is live, the detection image and BEV warp use the real camera frame.
- When `/zed/.../camera_info` is live, its intrinsics replace the HFOV fallback.

**As a real detector comes online, turn off the matching stub output**, so two
nodes never publish to one topic:

| Param | Default | Topic |
|---|---|---|
| `publish_yellow` | true | `/perception/lanes/yellow_mask` (when false, `mask_color` composites from the real topic) |
| `publish_mask_color` | true | `/perception/lanes/mask_color` |
| `publish_objects` | true | `/perception/objects/markers` |
| `publish_signals` | true | `/perception/signals/markers` |
| `publish_detection_image` | true | `/perception/detections/image` |
| `publish_bev` | true | `/perception/bev/image` |

Other params:

- `classes`: `all` or a comma list
- `rate_hz` (5)
- `light_period_s` (3)
- `watermark`
- `cam_x/y/z/pitch`, `ground_z`
- `bev_x_min/x_max/y_half/resolution`
- `fallback_hfov/width/height`

Example: `ros2 run autonav_viz perception_viz_stub --ros-args -p publish_objects:=false`.

## Notes

- **Wi-Fi bandwidth:** image displays use Best Effort QoS. The full-rate ZED image display is off by default because it's heavy over Wi-Fi; `/line_detection/debug/*` is throttled to 2 Hz by `line_detector`.
- **Costmap overlays:** the local costmap already contains the line layers. *Camera Line Costmap* draws `line_layer`'s own grid on top, so the pixel → planning-grid result stands out. Toggle it off to see only the fused map.
- **Grade map:** `/terrain/grade_map` is published in the upside-down `lidar_footprint` frame and is approximate by design (see `grade_detector`), so it's off by default.
