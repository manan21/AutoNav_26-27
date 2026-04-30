"""
Map Padder — seed-and-flood with distance-adaptive density.

Near the robot a solid disc of tiles is activated (fine detail for the
planner to navigate around obstacles).  Beyond that radius only a thin
corridor of tiles along the path/goal line is activated (coarse but
fast for distant waypoints).

Inactive cells inside the bounding box are set to LETHAL (100) so the
planner never expands them — functionally equivalent to a non-rectangular
costmap despite OccupancyGrid being rectangular.
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
        self.declare_parameter('output_resolution', 0.10)
        self.declare_parameter('near_radius_m', 15.0)     # dense disc radius
        self.declare_parameter('far_tile_size_m', 3.0)     # coarser tiles beyond disc
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_padded')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')

        self._tile = self.get_parameter('tile_size_m').value
        self._out_res = self.get_parameter('output_resolution').value
        self._near_r = self.get_parameter('near_radius_m').value
        self._far_tile = self.get_parameter('far_tile_size_m').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        self._tf_buf = Buffer()
        self._tf_lis = TransformListener(self._tf_buf, self)

        self._latest_goal = None
        self._latest_map = None
        self._plan_poses = None

        self._ds = None
        self._ds_ox = self._ds_oy = 0.0
        self._ds_w = self._ds_h = 0
        self._ds_res = 0.0
        self._slam_tiles = set()
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
            f'Map padder: tile={self._tile}m  far_tile={self._far_tile}m  '
            f'near_r={self._near_r}m  res={self._out_res}m')

    # ── tile helpers ─────────────────────────────────────────────────

    def _tile_of(self, x, y, size=None):
        s = size or self._tile
        return (math.floor(x / s), math.floor(y / s), s)

    @staticmethod
    def _ring(tiles):
        """Return tiles ∪ 8-neighbours (same tile size)."""
        out = set(tiles)
        for tx, ty, s in tiles:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    out.add((tx + dx, ty + dy, s))
        return out

    def _line_tiles(self, x0, y0, x1, y1, size):
        """Rasterise a line into tiles of given size."""
        tiles = set()
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / (size * 0.5)))
        inv = 1.0 / n
        for i in range(n + 1):
            t = i * inv
            tiles.add(self._tile_of(x0 + t * (x1 - x0),
                                    y0 + t * (y1 - y0), size))
        return tiles

    def _disc_tiles(self, cx, cy, radius, size):
        """All tiles of given size within radius of (cx, cy)."""
        tiles = set()
        r_tiles = math.ceil(radius / size)
        ctile = self._tile_of(cx, cy, size)
        ctx, cty = ctile[0], ctile[1]
        r_sq = (radius / size) ** 2
        for dx in range(-r_tiles, r_tiles + 1):
            for dy in range(-r_tiles, r_tiles + 1):
                if dx * dx + dy * dy <= r_sq:
                    tiles.add((ctx + dx, cty + dy, size))
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

        tile_near = self._tile
        tile_far = self._far_tile
        near_r = self._near_r

        # ── downsample SLAM (cached) ─────────────────────────────────
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

        # ── 1. seeds ─────────────────────────────────────────────────
        seeds = set()

        # SLAM tiles (fine, cached)
        if not self._slam_tiles:
            cpt = max(1, round(tile_near / res))
            for yr in range(0, self._ds_h, cpt):
                wy = self._ds_oy + yr * res
                for xr in range(0, self._ds_w, cpt):
                    wx = self._ds_ox + xr * res
                    blk = ds[yr:yr + cpt, xr:xr + cpt]
                    if np.any(blk != UNKNOWN):
                        self._slam_tiles.add(
                            self._tile_of(wx, wy, tile_near))
        seeds |= self._slam_tiles

        # Robot position
        rxy = self._robot_xy()
        if rxy:
            # Dense disc of fine tiles around robot
            seeds |= self._disc_tiles(rxy[0], rxy[1], near_r, tile_near)

        # Goal + line to goal
        if self._latest_goal:
            gx, gy = self._latest_goal
            seeds.add(self._tile_of(gx, gy, tile_far))

            origin = rxy or (self._ds_ox + self._ds_w * res / 2,
                             self._ds_oy + self._ds_h * res / 2)
            ox, oy = origin

            # Near portion of the line: fine tiles
            dist = math.hypot(gx - ox, gy - oy)
            if dist > 0:
                # Fraction of the line that's within the near radius
                near_frac = min(1.0, near_r / dist)
                mid_x = ox + near_frac * (gx - ox)
                mid_y = oy + near_frac * (gy - oy)

                if near_frac < 1.0:
                    # Near segment: fine tiles (already mostly covered by disc)
                    seeds |= self._line_tiles(
                        ox, oy, mid_x, mid_y, tile_near)
                    # Far segment: coarse tiles
                    seeds |= self._line_tiles(
                        mid_x, mid_y, gx, gy, tile_far)
                else:
                    # Entire line is within near radius
                    seeds |= self._line_tiles(ox, oy, gx, gy, tile_near)

        # Plan path tiles: near=fine, far=coarse
        if self._plan_poses and rxy:
            rx, ry = rxy
            near_r_sq = near_r * near_r
            for px, py in self._plan_poses:
                d_sq = (px - rx) ** 2 + (py - ry) ** 2
                if d_sq <= near_r_sq:
                    seeds.add(self._tile_of(px, py, tile_near))
                else:
                    seeds.add(self._tile_of(px, py, tile_far))

        # ── 2. flood one ring ────────────────────────────────────────
        active = self._ring(seeds)

        if active == self._prev_active and self._slam_tiles:
            return
        self._prev_active = set(active)

        if not active:
            return

        # ── 3. convert multi-size tiles to cell mask ─────────────────
        # Find world-space bounding box of all active tiles
        world_min_x = float('inf')
        world_min_y = float('inf')
        world_max_x = float('-inf')
        world_max_y = float('-inf')

        for tx, ty, s in active:
            x0 = tx * s
            y0 = ty * s
            world_min_x = min(world_min_x, x0)
            world_min_y = min(world_min_y, y0)
            world_max_x = max(world_max_x, x0 + s)
            world_max_y = max(world_max_y, y0 + s)

        # Snap to output resolution
        bb_ox = math.floor(world_min_x / res) * res
        bb_oy = math.floor(world_min_y / res) * res
        bb_w = min(5000, math.ceil((world_max_x - bb_ox) / res))
        bb_h = min(5000, math.ceil((world_max_y - bb_oy) / res))

        if bb_w <= 0 or bb_h <= 0:
            return

        # Paint active tiles into a cell-level boolean mask
        cmask = np.zeros((bb_h, bb_w), dtype=np.bool_)
        for tx, ty, s in active:
            # Convert tile world bounds to cell indices
            x0 = tx * s
            y0 = ty * s
            cx0 = max(0, round((x0 - bb_ox) / res))
            cy0 = max(0, round((y0 - bb_oy) / res))
            cx1 = min(bb_w, round((x0 + s - bb_ox) / res))
            cy1 = min(bb_h, round((y0 + s - bb_oy) / res))
            if cx1 > cx0 and cy1 > cy0:
                cmask[cy0:cy1, cx0:cx1] = True

        # ── 4. assemble grid ─────────────────────────────────────────
        grid = np.full((bb_h, bb_w), LETHAL, dtype=np.int8)
        grid[cmask] = UNKNOWN

        # Paste SLAM data into active cells
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

        # ── 5. publish ───────────────────────────────────────────────
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

        n_active = int(cmask.sum())
        self.get_logger().info(
            f'{len(active)} tiles  {n_active}/{bb_w * bb_h} active cells  '
            f'{bb_w}x{bb_h} ({bb_w * res:.0f}x{bb_h * res:.0f}m)')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapPadder())
    rclpy.shutdown()
