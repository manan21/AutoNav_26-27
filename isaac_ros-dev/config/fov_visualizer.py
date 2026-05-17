#!/usr/bin/env python3
"""FOV visualizer for RViz: camera trapezoid + lidar semicircle.

Publishes a MarkerArray on /fov_markers that shows where each sensor
adds/clears obstacles in the local costmap, anchored to base_link so
the markers follow the robot. Two markers:

  1. CAMERA TRAPEZOID — the ground polygon visible to the ZED's line
     detector, clipped by the geometry filter in line_detector.yaml
     (base_min_x_m, base_max_x_m, base_max_abs_y_m). Inside this
     polygon, the detector adds confirmed line voxels into the costmap
     (and, when the planned FOV-clear feature lands, will also clear
     stale ones).

  2. LIDAR SEMICIRCLE — the 270° horizontal coverage of the SICK
     multiScan100 / MRS-1xxx, capped at `lidar_range_m`. Inside this
     wedge the PCA pipeline both inserts obstacle points (via
     /scan_pca_filtered_points → ObstacleLayer) and ray-clears them
     (via /scan_pca_filtered_clear → the clear scan).

All geometry is computed in base_link frame so RViz applies the live
TF and the markers drag along with the robot. Run with:

  python3 fov_visualizer.py

No build step needed. The script uses only rclpy / standard message
types, so it works inside the container without rebuilding any
package.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# ── camera geometry — mirrors autonav_detection/config/line_detector.yaml
BASE_MIN_X_M = -0.25
BASE_MAX_X_M = 5.0
BASE_MAX_ABS_Y_M = 3.0
# ZED 2 HD720 horizontal FoV ≈ 100°. tan(50°) ≈ 1.19. Adjust if the
# launched resolution differs.
HALF_HFOV_TAN = math.tan(math.radians(100.0 / 2.0))

# ── lidar geometry — SICK multiScan100 / MRS-1xxx defaults
LIDAR_HALF_FOV_DEG = 135.0    # full 270° centered on forward
LIDAR_RANGE_M = 25.0
LIDAR_ARC_POINTS = 60

# ── markers
BASE_FRAME = "base_link"
PUBLISH_HZ = 5.0


def trapezoid_corners_base_link():
    """Return list of (x, y) corners of the camera FOV polygon on the
    ground in base_link frame. Polygon is the intersection of:
      • camera HFOV cone: |y| <= x * tan(half_hfov)
      • geometry filter:  x in [BASE_MIN_X_M, BASE_MAX_X_M],
                          |y| <= BASE_MAX_ABS_Y_M
    Returns a closed polygon (last point == first).
    """
    # Where the FoV cone hits the lateral cap |y| = BASE_MAX_ABS_Y_M.
    x_transition = BASE_MAX_ABS_Y_M / HALF_HFOV_TAN

    near_x = BASE_MIN_X_M
    near_y = max(0.001, near_x * HALF_HFOV_TAN) if near_x > 0 else 0.05

    pts = [
        (near_x, -near_y),
        (near_x, +near_y),
    ]
    if x_transition < BASE_MAX_X_M:
        pts += [
            (x_transition, +BASE_MAX_ABS_Y_M),
            (BASE_MAX_X_M, +BASE_MAX_ABS_Y_M),
            (BASE_MAX_X_M, -BASE_MAX_ABS_Y_M),
            (x_transition, -BASE_MAX_ABS_Y_M),
        ]
    else:
        far_y = BASE_MAX_X_M * HALF_HFOV_TAN
        pts += [
            (BASE_MAX_X_M, +far_y),
            (BASE_MAX_X_M, -far_y),
        ]
    pts.append(pts[0])  # close the polygon
    return pts


def lidar_arc_points_base_link():
    """Return list of (x, y) points tracing the SICK FoV semicircle in
    base_link frame, starting from the right-side end, sweeping
    counterclockwise through the front, ending at the left-side end,
    and closing back through the origin so the polygon is fillable in
    RViz if desired.
    """
    pts = [(0.0, 0.0)]
    start = math.radians(-LIDAR_HALF_FOV_DEG)
    stop = math.radians(+LIDAR_HALF_FOV_DEG)
    for i in range(LIDAR_ARC_POINTS + 1):
        t = start + (stop - start) * (i / LIDAR_ARC_POINTS)
        pts.append((LIDAR_RANGE_M * math.cos(t), LIDAR_RANGE_M * math.sin(t)))
    pts.append((0.0, 0.0))
    return pts


def make_polygon_marker(ns, mid, color, pts, line_width):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = line_width
    m.color = color
    m.pose.orientation.w = 1.0
    m.lifetime.sec = 0  # forever, replaced by next publish
    for (x, y) in pts:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.02   # just above ground so RViz draws over the costmap
        m.points.append(p)
    return m


class FOVVisualizer(Node):
    def __init__(self):
        super().__init__("fov_visualizer")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(MarkerArray, "/fov_markers", qos)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.cam_pts = trapezoid_corners_base_link()
        self.lidar_pts = lidar_arc_points_base_link()
        self.get_logger().info(
            f"FOV viz: camera polygon {len(self.cam_pts)} pts, "
            f"lidar arc {len(self.lidar_pts)} pts, "
            f"publishing /fov_markers @ {PUBLISH_HZ:.0f} Hz in {BASE_FRAME}"
        )

    def _tick(self):
        now = self.get_clock().now().to_msg()
        cam_color = ColorRGBA(r=0.10, g=0.85, b=1.00, a=0.95)   # cyan
        lidar_color = ColorRGBA(r=1.00, g=0.55, b=0.10, a=0.85) # orange
        cam = make_polygon_marker(
            "camera_fov", 0, cam_color, self.cam_pts, line_width=0.04)
        lidar = make_polygon_marker(
            "lidar_fov", 0, lidar_color, self.lidar_pts, line_width=0.04)
        cam.header.stamp = now
        lidar.header.stamp = now
        arr = MarkerArray()
        arr.markers.extend([cam, lidar])
        self.pub.publish(arr)


def main():
    rclpy.init()
    try:
        rclpy.spin(FOVVisualizer())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
