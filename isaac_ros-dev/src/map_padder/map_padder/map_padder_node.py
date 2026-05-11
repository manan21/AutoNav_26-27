"""
Map Padder — tight seed-and-flood.

Seeds (tiles of interest):
  • SLAM tiles   — every 1×1 m tile with real SLAM data
  • Robot tile   — single tile the robot occupies
  • Goal tile    — single tile the goal is in
  • Line tiles   — straight line from robot to goal
  • Path tiles   — tiles along the current /plan

Every seed gets its 8 neighbours activated (1-ring flood).
Everything else is LETHAL (100) — the planner can't enter it.

Result: tight walls hugging SLAM data and a narrow 3-tile-wide
corridor to distant goals.  As the robot moves and SLAM expands,
the active region grows organically.
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

# Maximum output grid side, in cells. At 0.10 m resolution that's 160 m
# (525 ft) — comfortably covers a 500 ft competition course with margin.
# Memory at MAX² × 1 byte = 2.56 MB per published grid. The _grid_buf
# allocated once in __init__ is sized to this.
MAX_GRID_SIDE = 1600


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

        self._ds = None
        self._ds_ox = self._ds_oy = 0.0
        self._ds_w = self._ds_h = 0
        self._ds_res = 0.0
        self._slam_tiles = set()
        self._prev_active = None

        # /plan arrives on every controller tick (~20 Hz); republishing
        # the whole padded grid that fast saturates global_costmap's
        # static_layer. We coalesce plan updates into a 2 Hz heartbeat
        # while /map and /goal still trigger immediate publishes.
        self._pending_plan_publish = False

        # Pre-allocated scratch buffer reused on every _publish so a
        # MAX × MAX (2.56 MB) ndarray isn't allocated per call. .fill()
        # is in-place; .reshape views are O(1). Sized to the worst case
        # so the slice grid = self._grid_buf[:bb_h, :bb_w] is always
        # valid after the clamp below.
        self._grid_buf = np.empty(
            (MAX_GRID_SIDE, MAX_GRID_SIDE), dtype=np.int8)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self._pub = self.create_publisher(OccupancyGrid, output_topic, map_qos)
        self.create_subscription(OccupancyGrid, input_topic, self._on_map, map_qos)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.create_subscription(Path, plan_topic, self._on_plan, 10)
        self.create_timer(0.5, self._plan_throttle_tick)

        # Publish an empty stub IMMEDIATELY so Nav2's static_layer has
        # something to latch onto during its configure() — without this,
        # if Nav2 comes up before slam_toolbox has inserted its first
        # keyframe (gated on robot motion via minimum_travel_distance /
        # minimum_travel_heading), map_padder never publishes and the
        # global_costmap is born empty. The first real /map arriving
        # overwrites this stub via the TRANSIENT_LOCAL latch.
        self._publish_initial_stub()

        self.get_logger().info(
            f'Map padder: tile={self._tile}m  res={self._out_res}m')

    # ── helpers ──────────────────────────────────────────────────────

    def _tile_of(self, x, y):
        t = self._tile
        return (math.floor(x / t), math.floor(y / t))

    @staticmethod
    def _ring(tiles):
        """Return tiles ∪ 8-neighbours."""
        out = set(tiles)
        for tx, ty in tiles:
            out.update((
                (tx-1, ty-1), (tx, ty-1), (tx+1, ty-1),
                (tx-1, ty),               (tx+1, ty),
                (tx-1, ty+1), (tx, ty+1), (tx+1, ty+1),
            ))
        return out

    def _line_tiles(self, x0, y0, x1, y1):
        """Rasterise a line into tiles at half-tile steps."""
        tiles = set()
        d = math.hypot(x1 - x0, y1 - y0)
        t = self._tile
        n = max(1, int(d / (t * 0.5)))
        inv = 1.0 / n
        for i in range(n + 1):
            f = i * inv
            tiles.add(self._tile_of(x0 + f*(x1-x0), y0 + f*(y1-y0)))
        return tiles

    @staticmethod
    def _downsample_max(grid, factor):
        h, w = grid.shape
        th, tw = (h // factor) * factor, (w // factor) * factor
        return grid[:th, :tw].reshape(
            th // factor, factor, tw // factor, factor).max(axis=(1, 3))

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
        # Defer to the throttle timer — Nav2 republishes /plan on every
        # controller tick and the padded grid only needs to track the
        # corridor at the costmap's own publish rate (~2 Hz).
        self._pending_plan_publish = True

    def _publish_initial_stub(self):
        """Emit a placeholder OccupancyGrid so subscribers configuring
        before SLAM's first /map don't see an empty topic. Size is
        50 m square at the configured output resolution — matches the
        global_costmap defaults in nav2_paramsv2.yaml so static_layer
        sees a sane stub. All cells are UNKNOWN; the first real /map
        replaces this entirely via TRANSIENT_LOCAL.
        """
        side_m = 50.0
        res = self._out_res if self._out_res > 0 else 0.10
        cells = int(side_m / res)
        stub = OccupancyGrid()
        stub.header.frame_id = 'map'
        stub.header.stamp = self.get_clock().now().to_msg()
        stub.info.resolution = res
        stub.info.width = cells
        stub.info.height = cells
        stub.info.origin.position.x = -side_m / 2.0
        stub.info.origin.position.y = -side_m / 2.0
        stub.info.origin.position.z = 0.0
        stub.info.origin.orientation.w = 1.0
        stub.data = array.array(
            'b',
            np.full(cells * cells, UNKNOWN, dtype=np.int8).tobytes(),
        )
        self._pub.publish(stub)
        self.get_logger().info(
            f'Initial /map_padded stub: {cells}x{cells} cells '
            f'({side_m:.0f}m square @ {res}m res, all UNKNOWN)')

    def _plan_throttle_tick(self):
        if self._pending_plan_publish and self._latest_map:
            self._publish(self._latest_map)
            self._pending_plan_publish = False

    # ── core ─────────────────────────────────────────────────────────

    def _publish(self, msg):
        slam_res = msg.info.resolution
        if slam_res <= 0:
            return

        tile = self._tile

        # ── downsample SLAM (cached) ─────────────────────────────────
        if self._ds is None:
            # np.frombuffer on the incoming array.array is zero-copy
            # (buffer protocol). The previous bytearray() wrap copied
            # the whole /map; on a 1600×1600 grid that's 2.56 MB per
            # /map arrival saved.
            raw = np.frombuffer(msg.data, dtype=np.int8
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

        # SLAM tiles (cached — recomputed only when /map changes)
        if not self._slam_tiles:
            cpt = max(1, round(tile / res))
            for yr in range(0, self._ds_h, cpt):
                wy = self._ds_oy + yr * res
                for xr in range(0, self._ds_w, cpt):
                    wx = self._ds_ox + xr * res
                    blk = ds[yr:yr + cpt, xr:xr + cpt]
                    if np.any(blk != UNKNOWN):
                        self._slam_tiles.add(self._tile_of(wx, wy))
        seeds |= self._slam_tiles

        # Robot tile
        rxy = self._robot_xy()
        if rxy:
            seeds.add(self._tile_of(*rxy))

        # Goal tile + straight line from robot to goal
        if self._latest_goal:
            gx, gy = self._latest_goal
            seeds.add(self._tile_of(gx, gy))
            origin = rxy or (self._ds_ox + self._ds_w * res / 2,
                             self._ds_oy + self._ds_h * res / 2)
            seeds |= self._line_tiles(origin[0], origin[1], gx, gy)

        # Plan path tiles
        if self._plan_poses:
            for px, py in self._plan_poses:
                seeds.add(self._tile_of(px, py))

        # ── 2. flood one ring of 8-neighbours ────────────────────────
        active = self._ring(seeds)

        if active == self._prev_active and self._slam_tiles:
            return
        self._prev_active = set(active)

        if not active:
            return

        # ── 3. bounding box ──────────────────────────────────────────
        min_tx = min(t[0] for t in active)
        max_tx = max(t[0] for t in active)
        min_ty = min(t[1] for t in active)
        max_ty = max(t[1] for t in active)

        bb_ox = min_tx * tile
        bb_oy = min_ty * tile
        cpt = max(1, round(tile / res))
        bb_w = (max_tx - min_tx + 1) * cpt
        bb_h = (max_ty - min_ty + 1) * cpt

        if bb_w > MAX_GRID_SIDE or bb_h > MAX_GRID_SIDE:
            self.get_logger().warn(f'Clamped {bb_w}x{bb_h}')
            bb_w = min(bb_w, MAX_GRID_SIDE)
            bb_h = min(bb_h, MAX_GRID_SIDE)

        # ── 4. tile mask → cell mask ─────────────────────────────────
        n_tx = max_tx - min_tx + 1
        n_ty = max_ty - min_ty + 1
        tmask = np.zeros((n_ty, n_tx), dtype=np.bool_)
        for tx, ty in active:
            tmask[ty - min_ty, tx - min_tx] = True

        cmask = np.repeat(np.repeat(tmask, cpt, axis=0),
                          cpt, axis=1)[:bb_h, :bb_w]

        # ── 5. build grid ────────────────────────────────────────────
        # Reuse the pre-allocated buffer instead of allocating a fresh
        # (bb_h, bb_w) ndarray per publish. .fill() is in-place.
        grid = self._grid_buf[:bb_h, :bb_w]
        grid.fill(LETHAL)
        grid[cmask] = UNKNOWN

        # Paste SLAM data into active corridor
        ox = round((self._ds_ox - bb_ox) / res)
        oy = round((self._ds_oy - bb_oy) / res)
        sy0, sx0 = max(0, -oy), max(0, -ox)
        dy0, dx0 = max(0, oy), max(0, ox)
        ch = min(self._ds_h - sy0, bb_h - dy0)
        cw = min(self._ds_w - sx0, bb_w - dx0)
        if ch > 0 and cw > 0:
            region = grid[dy0:dy0+ch, dx0:dx0+cw]
            slam = ds[sy0:sy0+ch, sx0:sx0+cw]
            mask = cmask[dy0:dy0+ch, dx0:dx0+cw]
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
        # frombytes pattern saves one alloc vs array.array(typecode, bytes)
        # — relevant at 1600×1600 where each publish's data field is 2.56 MB.
        out.data = array.array('b')
        out.data.frombytes(grid.tobytes())
        self._pub.publish(out)

        # Dropped n_active = int(cmask.sum()) from the log line — that
        # reduces a (bb_w × bb_h) bool array on every publish (millions
        # of cells at the worst case) purely to print a count.
        self.get_logger().info(
            f'{len(active)} tiles  '
            f'{bb_w}x{bb_h} ({bb_w*res:.0f}x{bb_h*res:.0f}m)')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapPadder())
    rclpy.shutdown()
