# autonav_viz

RViz config and launch file for the CV pipeline. This package does **not** do
perception. It only visualizes topics the stack already publishes.

## Run

```bash
# Laptop (native RViz, no build needed): uses cv_pipeline.rviz by default
./isaac_ros-dev/config/run-rviz.sh

# Or as a package (after colcon build)
ros2 launch autonav_viz viz.launch.py
ros2 launch autonav_viz viz.launch.py use_sim_time:=true    # rosbag replay (--clock)
```

The fixed frame is `odom`, and the default view is top-down, following
`base_link`.

## Displays

| Group | Topic | Type | Publisher | On by default |
|---|---|---|---|---|
| Robot | `/robot_description`, TF | RobotModel / TF | `robot_state_publisher` | yes |
| Lanes | `/line_detection/debug/overlay` | `sensor_msgs/Image` | `line_detector` (2 Hz) | yes |
| Lanes | `/line_detection/debug/mask` | `Image` (mono8) | `line_detector` | no |
| Lanes | `/line_detection/debug/raw` | `Image` | `line_detector` (2 Hz) | no |
| Lanes | `/zed/zed_node/rgb/color/rect/image` | `Image` | ZED (full rate) | no |
| Lanes | `/lines_pointcloud` | `PointCloud2` | `line_detector` | yes |
| Lanes | `/lidar_line_detection/debug/points` | `PointCloud2` | `lidar_line_detector` | yes |
| Obstacles | `/scan_pca_filtered_points` | `PointCloud2` | `grade_detector` | yes |
| Obstacles | `/scan_pca_filtered` | `LaserScan` | `grade_detector` | no |
| Costmaps and Path | `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 | yes |
| Costmaps and Path | `/line_costmap` | `OccupancyGrid` | `line_layer` | yes |
| Costmaps and Path | `/lidar_line_costmap` | `OccupancyGrid` | `lidar_line_layer` | no |
| Costmaps and Path | `/global_costmap/costmap` | `OccupancyGrid` | Nav2 | no |
| Costmaps and Path | `/plan` | `nav_msgs/Path` | Nav2 planner | yes |

## Notes

- **Wi-Fi bandwidth:** image displays use Best Effort QoS. The full-rate ZED
  image display is off by default because it's heavy over Wi-Fi.
- **Costmap overlays:** the local costmap already contains the line layers.
  *Camera Line Costmap* draws `line_layer`'s own grid on top so the camera's
  contribution stands out. Toggle it off to see only the fused map.
