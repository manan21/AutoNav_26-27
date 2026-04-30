"""
Map Padder Node — seed-and-flood tile expansion.

Divides the world into a 1 × 1 m tile grid.  Tiles are activated by seeds:

  - SLAM data tiles   — every tile that overlaps known SLAM occupancy
  - Robot tile         — the tile the robot is in (+ 1-ring neighbours)
  - Goal tile          — the tile the goal is in  (+ 1-ring neighbours)
  - Seed line tiles    — tiles along the straight line from robot to goal
  - Plan path tiles    — tiles along the current /plan path

Every active tile also activates its 8 immediate neighbours (1-ring flood).
The output map is the tight bounding box of all active tiles, filled with
SLAM data where available and -1 (unknown) elsewhere.

For a diagonal GPS goal 200 m away this produces ~600 active tiles instead
of the 40 000 a bounding-box approach would need.

Serialisation uses array.array (C-level memcpy) and the SLAM grid is
downsampled to a configurable output resolution (default 0.10 m).
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

        self.declare_parameter('tile_size_m', 1.0)
        self.declare_parameter('output_resolution', 0.10)
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_padded')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')

        self._tile = self.get_parameter('tile_size_m').value
        self._output_res = self.get_parameter('output_resolution').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # State
        self._latest_goal = None
        self._latest_map = None
        self._plan_poses = None      # list of (x, y) sampled from /plan

        # Cached downsampled SLAM data
        self._ds_slam = None
        self._ds_ox = 0.0
        self._ds_oy = 0.0
        self._ds_w = 0
        self._ds_h = 0
        self._ds_res = 0.0

        # Cached SLAM tile set so we don't recompute every publish
        self._slam_tiles = set()

        # Previous active tile set — skip republish if unchanged
        self._prev_active = None

        # QoS
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
            f'Map padder ready  tile={self._tile}m  '
            f'out_res={self._output_res}m  '
            f'{input_topic} -> {output_topic}')

    # ------------------------------------------------------------------
    # Tile helpers
    # ------------------------------------------------------------------

    def _world_to_tile(self, x, y):
        """Return the (tx, ty) integer tile index for a world coordinate."""
        return (math.floor(x / self._tile),
                math.floor(y / self._tile))

    def _ring(self, tiles):
        """Return tiles ∪ all 8-neighbours of every tile in the set."""
        expanded = set(tiles)
        for (tx, ty) in tiles:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    expanded.add((tx + dx, ty + dy))
        return expanded

    def _line_tiles(self, x0, y0, x1, y1):
        """Rasterise a world-space line segment into tile indices."""
        tiles = set()
        dist = math.hypot(x1 - x0, y1 - y0)
        tile = self._tile
        # Step in half-tile increments to avoid gaps
        n_steps = max(1, int(dist / (tile * 0.5)))
        for i in range(n_steps + 1):
            t = i / n_steps
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            tiles.add(self._world_to_tile(x, y))
        return tiles

    # ------------------------------------------------------------------
    # Downsampling
    # ------------------------------------------------------------------

    @staticmethod
    def _downsample(data_2d, factor):
        """Conservative downsample: max over each factor×factor block."""
        h, w = data_2d.shape
        th = (h // factor) * factor
        tw = (w // factor) * factor
        cropped = data_2d[:th, :tw]
        return cropped.reshape(
            th // factor, factor, tw // factor, factor
        ).max(axis=(1, 3))

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------

    def _get_robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_goal(self, msg: PoseStamped):
        self._latest_goal = (msg.pose.position.x, msg.pose.position.y)
        self._plan_poses = None  # stale
        self.get_logger().info(
            f'Goal ({self._latest_goal[0]:.1f}, {self._latest_goal[1]:.1f})')
        if self._latest_map is not None:
            self._pad_and_publish(self._latest_map)

    def _on_plan(self, msg: Path):
        if len(msg.poses) == 0:
            return
        # Sample path sparsely — one point per ~tile
        step = max(1, len(msg.poses) // 500)
        poses = [(msg.poses[i].pose.position.x,
                  msg.poses[i].pose.position.y)
                 for i in range(0, len(msg.poses), step)]
        poses.append((msg.poses[-1].pose.position.x,
                       msg.poses[-1].pose.position.y))
        self._plan_poses = poses
        if self._latest_map is not None:
            self._pad_and_publish(self._latest_map)

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg
        self._ds_slam = None      # invalidate downsample cache
        self._slam_tiles = set()  # invalidate tile cache
        self._pad_and_publish(msg)

    # ------------------------------------------------------------------
    # Core: seed → flood → bounding-box → publish
    # ------------------------------------------------------------------

    def _pad_and_publish(self, msg: OccupancyGrid):
        slam_res = msg.info.resolution
        if slam_res <= 0.0:
            return

        tile = self._tile

        # --- downsample SLAM (cached) ------------------------------------
        if self._ds_slam is None:
            orig = np.frombuffer(
                bytearray(msg.data), dtype=np.int8
            ).reshape(msg.info.height, msg.info.width)

            if (self._output_res > 0.0
                    and self._output_res > slam_res * 1.5):
                factor = max(1, round(self._output_res / slam_res))
                self._ds_res = slam_res * factor
                self._ds_slam = self._downsample(orig, factor)
            else:
                self._ds_res = slam_res
                self._ds_slam = orig.copy()

            self._ds_ox = msg.info.origin.position.x
            self._ds_oy = msg.info.origin.position.y
            self._ds_h, self._ds_w = self._ds_slam.shape

        ds = self._ds_slam
        out_res = self._ds_res

        # SLAM extent
        slam_min_x = self._ds_ox
        slam_min_y = self._ds_oy
        slam_max_x = self._ds_ox + self._ds_w * out_res
        slam_max_y = self._ds_oy + self._ds_h * out_res

        # =================================================================
        # 1) Collect seed tiles
        # =================================================================
        seeds = set()

        # SLAM tiles (cached — only recomputed when /map changes)
        if not self._slam_tiles:
            # Walk the downsampled grid in tile-sized steps and mark any
            # tile that contains at least one non-unknown cell.
            cells_per_tile = max(1, round(tile / out_res))
            for ty_off in range(0, self._ds_h, cells_per_tile):
                wy = self._ds_oy + ty_off * out_res
                ty = math.floor(wy / tile)
                for tx_off in range(0, self._ds_w, cells_per_tile):
                    wx = self._ds_ox + tx_off * out_res
                    tx = math.floor(wx / tile)
                    # Check a small block for any non-unknown cell
                    block = ds[ty_off:ty_off + cells_per_tile,
                               tx_off:tx_off + cells_per_tile]
                    if np.any(block != -1):
                        self._slam_tiles.add((tx, ty))
            self.get_logger().debug(
                f'SLAM covers {len(self._slam_tiles)} tiles')

        seeds.update(self._slam_tiles)

        # Robot tile
        robot_pos = self._get_robot_position()
        if robot_pos is not None:
            seeds.add(self._world_to_tile(*robot_pos))

        # Goal tile + seed line from robot (or SLAM centre) to goal
        if self._latest_goal is not None:
            gx, gy = self._latest_goal
            seeds.add(self._world_to_tile(gx, gy))

            if robot_pos is not None:
                seeds.update(self._line_tiles(
                    robot_pos[0], robot_pos[1], gx, gy))
            else:
                cx = (slam_min_x + slam_max_x) / 2.0
                cy = (slam_min_y + slam_max_y) / 2.0
                seeds.update(self._line_tiles(cx, cy, gx, gy))

        # Plan path tiles
        if self._plan_poses is not None:
            for px, py in self._plan_poses:
                seeds.add(self._world_to_tile(px, py))

        # =================================================================
        # 2) Flood: 1-ring around every seed
        # =================================================================
        active = self._ring(seeds)

        # Skip republish if tile set hasn't changed (unless SLAM updated)
        if active == self._prev_active and self._slam_tiles:
            return
        self._prev_active = active

        # =================================================================
        # 3) Bounding box of active tiles
        # =================================================================
        if not active:
            return

        min_tx = min(t[0] for t in active)
        max_tx = max(t[0] for t in active)
        min_ty = min(t[1] for t in active)
        max_ty = max(t[1] for t in active)

        bb_ox = min_tx * tile
        bb_oy = min_ty * tile
        bb_w = round(((max_tx - min_tx + 1) * tile) / out_res)
        bb_h = round(((max_ty - min_ty + 1) * tile) / out_res)

        # Sanity clamp
        MAX_DIM = 5000
        if bb_w > MAX_DIM or bb_h > MAX_DIM:
            self.get_logger().warn(
                f'Clamped {bb_w}x{bb_h} to {MAX_DIM}x{MAX_DIM}')
            bb_w = min(bb_w, MAX_DIM)
            bb_h = min(bb_h, MAX_DIM)

        # =================================================================
        # 4) Build output grid — unknown everywhere, SLAM where available
        # =================================================================
        padded = np.full((bb_h, bb_w), -1, dtype=np.int8)

        # Copy downsampled SLAM into the correct position
        off_x = round((self._ds_ox - bb_ox) / out_res)
        off_y = round((self._ds_oy - bb_oy) / out_res)

        src_y0 = max(0, -off_y)
        src_x0 = max(0, -off_x)
        dst_y0 = max(0, off_y)
        dst_x0 = max(0, off_x)
        copy_h = min(self._ds_h - src_y0, bb_h - dst_y0)
        copy_w = min(self._ds_w - src_x0, bb_w - dst_x0)

        if copy_h > 0 and copy_w > 0:
            padded[dst_y0:dst_y0 + copy_h,
                   dst_x0:dst_x0 + copy_w] = ds[src_y0:src_y0 + copy_h,
                                                  src_x0:src_x0 + copy_w]

        # =================================================================
        # 5) Mask out inactive tiles — set them back to -1 so the planner
        #    doesn't waste time on unused bounding-box corners.
        # =================================================================
        cells_per_tile = max(1, round(tile / out_res))
        # Build a mask grid at tile resolution
        n_tx = max_tx - min_tx + 1
        n_ty = max_ty - min_ty + 1
        tile_mask = np.zeros((n_ty, n_tx), dtype=np.bool_)
        for (tx, ty) in active:
            tile_mask[ty - min_ty, tx - min_tx] = True

        # Up-scale tile mask to cell resolution and apply
        cell_mask = np.repeat(
            np.repeat(tile_mask, cells_per_tile, axis=0),
            cells_per_tile, axis=1)
        # Trim to exact output dimensions (rounding may differ)
        cell_mask = cell_mask[:bb_h, :bb_w]
        padded[~cell_mask] = -1

        # =================================================================
        # 6) Publish
        # =================================================================
        out = OccupancyGrid()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = out_res
        out.info.width = bb_w
        out.info.height = bb_h
        out.info.origin.position.x = bb_ox
        out.info.origin.position.y = bb_oy
        out.info.origin.position.z = msg.info.origin.position.z
        out.info.origin.orientation = msg.info.origin.orientation
        out.data = array.array('b', padded.flatten().tobytes())

        self._pub.publish(out)

        size_x = bb_w * out_res
        size_y = bb_h * out_res
        n_active = len(active)
        self.get_logger().info(
            f'{n_active} tiles -> {bb_w}x{bb_h} cells '
            f'({size_x:.0f}x{size_y:.0f}m @ {out_res:.2f}m/cell)'
            f'{" [path]" if self._plan_poses else ""}'
            f'{" [goal]" if self._latest_goal else ""}')


def main(args=None):
    rclpy.init(args=args)
    node = MapPadder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
