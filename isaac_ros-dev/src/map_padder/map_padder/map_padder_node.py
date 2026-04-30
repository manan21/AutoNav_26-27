"""
Map Padder Node — tile-based expansion, optimised for distant GPS goals.

Subscribes to the SLAM /map and republishes a padded version on /map_padded.

Key optimisations over the naive approach:
  1. Output resolution is configurable (default 0.10 m) — SLAM data is
     downsampled before padding so the global costmap works on 4–16×
     fewer cells.
  2. Serialisation uses array.array (C-level memcpy) instead of
     .tolist() which creates millions of Python objects.
  3. The padded grid is cached and extended incrementally when bounds
     grow, instead of rebuilt from scratch.
  4. Tiles expand reactively: SLAM bounds, robot, goal, and the
     current /plan path — nothing more.
"""

import array
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException


class MapPadder(Node):

    def __init__(self):
        super().__init__('map_padder')

        # --- parameters ---------------------------------------------------
        self.declare_parameter('tile_size_m', 1.0)
        self.declare_parameter('robot_buffer_tiles', 2)
        self.declare_parameter('goal_buffer_tiles', 2)
        self.declare_parameter('path_buffer_tiles', 1)
        self.declare_parameter('output_resolution', 0.10)  # metres; 0 = use SLAM res
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_padded')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')

        self._tile = self.get_parameter('tile_size_m').value
        self._robot_buf = self.get_parameter('robot_buffer_tiles').value
        self._goal_buf = self.get_parameter('goal_buffer_tiles').value
        self._path_buf = self.get_parameter('path_buffer_tiles').value
        self._output_res = self.get_parameter('output_resolution').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        # --- TF ------------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- state ---------------------------------------------------------
        self._latest_goal = None        # (x, y) in map frame
        self._latest_map = None         # OccupancyGrid
        self._path_bounds = None        # (min_x, min_y, max_x, max_y)
        self._prev_bounds = None        # (ox, oy, w, h) of last publish

        # Cached downsampled SLAM data and its info
        self._ds_slam = None            # numpy int8 array at output res
        self._ds_slam_ox = 0.0
        self._ds_slam_oy = 0.0
        self._ds_slam_w = 0
        self._ds_slam_h = 0

        # --- pub / sub -----------------------------------------------------
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._pub = self.create_publisher(OccupancyGrid, output_topic, map_qos)
        self._sub = self.create_subscription(
            OccupancyGrid, input_topic, self._on_map, map_qos)
        self._goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal, 10)
        self._plan_sub = self.create_subscription(
            Path, plan_topic, self._on_plan, 10)

        self.get_logger().info(
            f'Map padder ready  tile={self._tile}m  out_res={self._output_res}m  '
            f'{input_topic} -> {output_topic}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _snap_lo(self, v):
        return math.floor(v / self._tile) * self._tile

    def _snap_hi(self, v):
        return math.ceil(v / self._tile) * self._tile

    def _get_robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    @staticmethod
    def _downsample(data_2d, factor):
        """Conservative downsample: take max over each factor×factor block.

        max gives: occupied (100) > free (0) > unknown (-1 = int8 min).
        So any occupied fine cell → occupied coarse cell, etc.
        """
        h, w = data_2d.shape
        # Truncate to exact multiple of factor
        th = (h // factor) * factor
        tw = (w // factor) * factor
        cropped = data_2d[:th, :tw]
        return cropped.reshape(th // factor, factor,
                               tw // factor, factor).max(axis=(1, 3))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_goal(self, msg: PoseStamped):
        self._latest_goal = (msg.pose.position.x, msg.pose.position.y)
        self._path_bounds = None  # stale — new plan incoming
        self.get_logger().info(
            f'Goal ({self._latest_goal[0]:.1f}, {self._latest_goal[1]:.1f})')
        if self._latest_map is not None:
            self._pad_and_publish(self._latest_map)

    def _on_plan(self, msg: Path):
        if len(msg.poses) == 0:
            return

        buf = self._path_buf * self._tile
        xs = []
        ys = []
        # Sample every ~1 m along the path (skip dense interior poses)
        step = max(1, len(msg.poses) // 300)
        for i in range(0, len(msg.poses), step):
            xs.append(msg.poses[i].pose.position.x)
            ys.append(msg.poses[i].pose.position.y)
        # Always include last pose
        xs.append(msg.poses[-1].pose.position.x)
        ys.append(msg.poses[-1].pose.position.y)

        new_pb = (
            self._snap_lo(min(xs) - buf),
            self._snap_lo(min(ys) - buf),
            self._snap_hi(max(xs) + buf),
            self._snap_hi(max(ys) + buf),
        )
        if new_pb != self._path_bounds:
            self._path_bounds = new_pb
            if self._latest_map is not None:
                self._pad_and_publish(self._latest_map)

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg
        self._ds_slam = None  # invalidate cache
        self._pad_and_publish(msg)

    # ------------------------------------------------------------------
    # Core
    # ------------------------------------------------------------------

    def _pad_and_publish(self, msg: OccupancyGrid):
        slam_res = msg.info.resolution
        if slam_res <= 0.0:
            return

        # Decide output resolution
        if self._output_res > 0.0 and self._output_res > slam_res * 1.5:
            out_res = self._output_res
            factor = max(1, round(out_res / slam_res))
            out_res = slam_res * factor  # exact multiple
        else:
            out_res = slam_res
            factor = 1

        # --- downsample SLAM data (cached) --------------------------------
        if self._ds_slam is None:
            orig = np.frombuffer(
                bytearray(msg.data), dtype=np.int8
            ).reshape(msg.info.height, msg.info.width)

            if factor > 1:
                self._ds_slam = self._downsample(orig, factor)
            else:
                self._ds_slam = orig.copy()

            self._ds_slam_ox = msg.info.origin.position.x
            self._ds_slam_oy = msg.info.origin.position.y
            self._ds_slam_w = self._ds_slam.shape[1]
            self._ds_slam_h = self._ds_slam.shape[0]

        ds = self._ds_slam
        ds_ox = self._ds_slam_ox
        ds_oy = self._ds_slam_oy
        ds_w = self._ds_slam_w
        ds_h = self._ds_slam_h

        # SLAM extent in world coords
        slam_min_x = ds_ox
        slam_min_y = ds_oy
        slam_max_x = ds_ox + ds_w * out_res
        slam_max_y = ds_oy + ds_h * out_res

        tile = self._tile

        # --- compute needed bounds ----------------------------------------
        need_min_x = self._snap_lo(slam_min_x)
        need_min_y = self._snap_lo(slam_min_y)
        need_max_x = self._snap_hi(slam_max_x)
        need_max_y = self._snap_hi(slam_max_y)

        # Robot
        robot_pos = self._get_robot_position()
        if robot_pos is not None:
            rx, ry = robot_pos
            buf = self._robot_buf * tile
            need_min_x = min(need_min_x, self._snap_lo(rx - buf))
            need_max_x = max(need_max_x, self._snap_hi(rx + buf))
            need_min_y = min(need_min_y, self._snap_lo(ry - buf))
            need_max_y = max(need_max_y, self._snap_hi(ry + buf))

        # Goal
        if self._latest_goal is not None:
            gx, gy = self._latest_goal
            gbuf = self._goal_buf * tile
            need_min_x = min(need_min_x, self._snap_lo(gx - gbuf))
            need_max_x = max(need_max_x, self._snap_hi(gx + gbuf))
            need_min_y = min(need_min_y, self._snap_lo(gy - gbuf))
            need_max_y = max(need_max_y, self._snap_hi(gy + gbuf))

        # Planned path
        if self._path_bounds is not None:
            pb = self._path_bounds
            need_min_x = min(need_min_x, pb[0])
            need_min_y = min(need_min_y, pb[1])
            need_max_x = max(need_max_x, pb[2])
            need_max_y = max(need_max_y, pb[3])

        # --- cell dimensions at output resolution ------------------------
        pad_ox = need_min_x
        pad_oy = need_min_y
        pad_w = round((need_max_x - need_min_x) / out_res)
        pad_h = round((need_max_y - need_min_y) / out_res)

        # Sanity clamp — never publish more than 5000 × 5000 (25 M cells)
        MAX_DIM = 5000
        if pad_w > MAX_DIM or pad_h > MAX_DIM:
            self.get_logger().warn(
                f'Clamping padded map from {pad_w}x{pad_h} to {MAX_DIM}x{MAX_DIM}')
            pad_w = min(pad_w, MAX_DIM)
            pad_h = min(pad_h, MAX_DIM)

        # Where downsampled SLAM data sits inside the padded grid
        off_x = round((ds_ox - pad_ox) / out_res)
        off_y = round((ds_oy - pad_oy) / out_res)

        # No padding needed?
        if (pad_w == ds_w and pad_h == ds_h and off_x == 0 and off_y == 0):
            out = self._build_msg(msg, ds.flatten(), out_res,
                                  pad_ox, pad_oy, pad_w, pad_h)
            self._pub.publish(out)
            self._prev_bounds = (pad_ox, pad_oy, pad_w, pad_h)
            return

        # --- build padded grid (unknown = -1) -----------------------------
        padded = np.full((pad_h, pad_w), -1, dtype=np.int8)

        # Clamp SLAM placement to valid region
        src_y0 = max(0, -off_y)
        src_x0 = max(0, -off_x)
        dst_y0 = max(0, off_y)
        dst_x0 = max(0, off_x)
        copy_h = min(ds_h - src_y0, pad_h - dst_y0)
        copy_w = min(ds_w - src_x0, pad_w - dst_x0)

        if copy_h > 0 and copy_w > 0:
            padded[dst_y0:dst_y0 + copy_h,
                   dst_x0:dst_x0 + copy_w] = ds[src_y0:src_y0 + copy_h,
                                                  src_x0:src_x0 + copy_w]

        out = self._build_msg(msg, padded.flatten(), out_res,
                              pad_ox, pad_oy, pad_w, pad_h)
        self._pub.publish(out)
        self._prev_bounds = (pad_ox, pad_oy, pad_w, pad_h)

        size_x = pad_w * out_res
        size_y = pad_h * out_res
        self.get_logger().info(
            f'{ds_w}x{ds_h} -> {pad_w}x{pad_h} ({size_x:.0f}x{size_y:.0f}m '
            f'@ {out_res:.2f}m/cell = {pad_w * pad_h / 1e6:.1f}M cells)'
            f'{" [path]" if self._path_bounds else ""}'
            f'{" [goal]" if self._latest_goal else ""}')

    def _build_msg(self, ref_msg, flat_data, res, ox, oy, w, h):
        """Construct an OccupancyGrid from a flat int8 numpy array."""
        out = OccupancyGrid()
        out.header.frame_id = ref_msg.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = res
        out.info.width = w
        out.info.height = h
        out.info.origin.position.x = ox
        out.info.origin.position.y = oy
        out.info.origin.position.z = ref_msg.info.origin.position.z
        out.info.origin.orientation = ref_msg.info.origin.orientation
        # array.array('b', bytes) is C-level memcpy — orders of magnitude
        # faster than .tolist() which creates N Python int objects.
        out.data = array.array('b', flat_data.tobytes())
        return out


def main(args=None):
    rclpy.init(args=args)
    node = MapPadder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
