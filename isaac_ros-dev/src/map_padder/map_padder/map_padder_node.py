"""
Map Padder — seed-and-flood tile expansion.

World is divided into 1×1 m tiles.  Only tiles of interest (and their
8 immediate neighbours) are marked traversable.  Everything else in the
bounding box is lethal (100), so the planner never expands it.

Seeds (tiles of interest):
  • Every tile that overlaps SLAM data
  • The robot's tile
  • The goal's tile
  • Every tile the straight line from robot→goal passes through
  • Every tile the current /plan path passes through

OccupancyGrid is always rectangular, so the output is the tight bounding
box of all active tiles.  Inactive cells inside the box are lethal — the
planner treats them as walls, giving the same effect as a non-rectangular
grid.  A coarser output resolution (default 0.25 m) keeps the bounding
box small even for distant goals.
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

LETHAL = np.int8(100)
UNKNOWN = np.int8(-1)


class MapPadder(Node):

    def __init__(self):
        super().__init__('map_padder')

        self.declare_parameter('tile_size_m', 1.0)
        self.declare_parameter('output_resolution', 0.25)
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_padded')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')

        self._tile = self.get_parameter('tile_size_m').value
        self._out_res = self.get_parameter('output_resolution').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        self._tf_buf = Buffer()
        self._tf_lis = TransformListener(self._tf_buf, self)

        self._latest_goal = None
        self._latest_map = None
        self._plan_poses = None

        # Cached downsample of SLAM grid + which tiles it covers
        self._ds = None
        self._ds_ox = self._ds_oy = 0.0
        self._ds_w = self._ds_h = 0
        self._ds_res = 0.0
        self._slam_tiles = set()

        # Dedup — skip republish when active tile set is unchanged
        self._prev_active = None

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self._pub = self.create_publisher(OccupancyGrid, output_topic, map_qos)
        self.create_subscription(OccupancyGrid, input_topic, self._on_map, map_qos)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.create_subscription(Path, plan_topic, self._on_plan, 10)

        self.get_logger().info(
            f'Map padder: tile={self._tile}m  res={self._out_res}m  '
            f'{input_topic}->{output_topic}')

    # ── tile helpers ─────────────────────────────────────────────────

    def _tile_of(self, x, y):
        t = self._tile
        return (math.floor(x / t), math.floor(y / t))

    @staticmethod
    def _flood_one_ring(seeds):
        """Return seeds ∪ 8-neighbours of every seed."""
        out = set(seeds)
        for tx, ty in seeds:
            out.update((
                (tx - 1, ty - 1), (tx, ty - 1), (tx + 1, ty - 1),
                (tx - 1, ty),                    (tx + 1, ty),
                (tx - 1, ty + 1), (tx, ty + 1), (tx + 1, ty + 1),
            ))
        return out

    def _bresenham_tiles(self, x0, y0, x1, y1):
        """Walk a line at half-tile steps and collect every tile touched."""
        tiles = set()
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / (self._tile * 0.5)))
        inv = 1.0 / n
        for i in range(n + 1):
            t = i * inv
            tiles.add(self._tile_of(x0 + t * (x1 - x0),
                                    y0 + t * (y1 - y0)))
        return tiles

    # ── downsample ───────────────────────────────────────────────────

    @staticmethod
    def _downsample_max(grid, factor):
        h, w = grid.shape
        th, tw = (h // factor) * factor, (w // factor) * factor
        return grid[:th, :tw].reshape(
            th // factor, factor, tw // factor, factor).max(axis=(1, 3))

    # ── tf ───────────────────────────────────────────────────────────

    def _robot_xy(self):
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    # ── callbacks ────────────────────────────────────────────────────

    def _on_map(self, msg):
        self._latest_map = msg
        self._ds = None
        self._slam_tiles = set()
        self._publish(msg)

    def _on_goal(self, msg):
        self._latest_goal = (msg.pose.position.x, msg.pose.position.y)
        self._plan_poses = None
        self.get_logger().info(
            f'Goal ({self._latest_goal[0]:.1f}, {self._latest_goal[1]:.1f})')
        if self._latest_map:
            self._publish(self._latest_map)

    def _on_plan(self, msg):
        if not msg.poses:
            return
        step = max(1, len(msg.poses) // 500)
        self._plan_poses = [
            (msg.poses[i].pose.position.x, msg.poses[i].pose.position.y)
            for i in range(0, len(msg.poses), step)]
        self._plan_poses.append(
            (msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y))
        if self._latest_map:
            self._publish(self._latest_map)

    # ── core ─────────────────────────────────────────────────────────

    def _publish(self, msg):
        slam_res = msg.info.resolution
        if slam_res <= 0:
            return

        # ── 0. downsample SLAM (cached) ──────────────────────────────
        if self._ds is None:
            raw = np.frombuffer(bytearray(msg.data), dtype=np.int8
                                ).reshape(msg.info.height, msg.info.width)
            factor = max(1, round(self._out_res / slam_res)) \
                if self._out_res > slam_res * 1.5 else 1
            self._ds_res = slam_res * factor
            self._ds = self._downsample_max(raw, factor) \
                if factor > 1 else raw.copy()
            self._ds_ox = msg.info.origin.position.x
            self._ds_oy = msg.info.origin.position.y
            self._ds_h, self._ds_w = self._ds.shape

        ds = self._ds
        res = self._ds_res
        tile = self._tile

        # ── 1. collect seed tiles ────────────────────────────────────
        seeds = set()

        #  SLAM tiles (cached)
        if not self._slam_tiles:
            cpt = max(1, round(tile / res))          # cells per tile
            for yr in range(0, self._ds_h, cpt):
                wy = self._ds_oy + yr * res
                for xr in range(0, self._ds_w, cpt):
                    wx = self._ds_ox + xr * res
                    blk = ds[yr:yr + cpt, xr:xr + cpt]
                    if np.any(blk != UNKNOWN):
                        self._slam_tiles.add(self._tile_of(wx, wy))
        seeds |= self._slam_tiles

        #  Robot
        rxy = self._robot_xy()
        if rxy:
            seeds.add(self._tile_of(*rxy))

        #  Goal + line robot→goal
        if self._latest_goal:
            gx, gy = self._latest_goal
            seeds.add(self._tile_of(gx, gy))
            if rxy:
                seeds |= self._bresenham_tiles(rxy[0], rxy[1], gx, gy)
            else:
                cx = self._ds_ox + self._ds_w * res / 2
                cy = self._ds_oy + self._ds_h * res / 2
                seeds |= self._bresenham_tiles(cx, cy, gx, gy)

        #  Plan path
        if self._plan_poses:
            for px, py in self._plan_poses:
                seeds.add(self._tile_of(px, py))

        # ── 2. flood one ring ────────────────────────────────────────
        active = self._flood_one_ring(seeds)

        if active == self._prev_active and self._slam_tiles:
            return                         # nothing changed
        self._prev_active = set(active)    # store a copy

        if not active:
            return

        # ── 3. bounding box ──────────────────────────────────────────
        min_tx = min(t[0] for t in active)
        max_tx = max(t[0] for t in active)
        min_ty = min(t[1] for t in active)
        max_ty = max(t[1] for t in active)

        bb_ox = min_tx * tile
        bb_oy = min_ty * tile
        n_tx = max_tx - min_tx + 1
        n_ty = max_ty - min_ty + 1
        cpt = max(1, round(tile / res))
        bb_w = n_tx * cpt
        bb_h = n_ty * cpt

        MAX = 5000
        if bb_w > MAX or bb_h > MAX:
            self.get_logger().warn(f'Clamped {bb_w}x{bb_h}')
            bb_w, bb_h = min(bb_w, MAX), min(bb_h, MAX)

        # ── 4. build tile-level active mask, upscale to cell grid ────
        tmask = np.zeros((n_ty, n_tx), dtype=np.bool_)
        for tx, ty in active:
            tmask[ty - min_ty, tx - min_tx] = True

        cmask = np.repeat(np.repeat(tmask, cpt, axis=0),
                          cpt, axis=1)[:bb_h, :bb_w]

        # ── 5. assemble output grid ──────────────────────────────────
        #   inactive = LETHAL  (planner can't enter)
        #   active   = UNKNOWN (traversable corridor)
        #   SLAM     = real data overlaid
        grid = np.full((bb_h, bb_w), LETHAL, dtype=np.int8)
        grid[cmask] = UNKNOWN

        # paste SLAM data into the active corridor
        ox = round((self._ds_ox - bb_ox) / res)
        oy = round((self._ds_oy - bb_oy) / res)
        sy0, sx0 = max(0, -oy), max(0, -ox)
        dy0, dx0 = max(0, oy), max(0, ox)
        ch = min(self._ds_h - sy0, bb_h - dy0)
        cw = min(self._ds_w - sx0, bb_w - dx0)
        if ch > 0 and cw > 0:
            region = grid[dy0:dy0 + ch, dx0:dx0 + cw]
            slam = ds[sy0:sy0 + ch, sx0:sx0 + cw]
            mask = cmask[dy0:dy0 + ch, dx0:dx0 + cw]
            region[mask] = slam[mask]

        # ── 6. publish ───────────────────────────────────────────────
        out = OccupancyGrid()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = res
        out.info.width = bb_w
        out.info.height = bb_h
        out.info.origin.position.x = bb_ox
        out.info.origin.position.y = bb_oy
        out.info.origin.position.z = msg.info.origin.position.z
        out.info.origin.orientation = msg.info.origin.orientation
        out.data = array.array('b', grid.ravel().tobytes())
        self._pub.publish(out)

        self.get_logger().info(
            f'{len(active)} active tiles  {bb_w}x{bb_h} cells '
            f'({bb_w * res:.0f}x{bb_h * res:.0f}m @ {res:.2f}m)')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapPadder())
    rclpy.shutdown()
