"""
Map Padder — corridor + wall + buffer geometry for the global costmap.

Seeds (tiles of interest):
  • Robot window tiles — the local-costmap footprint around base_link
  • Goal tile          — single tile the goal is in
  • Line tiles         — straight line from robot to goal
  • Path tiles         — tiles along the current /plan

Every seed gets its 8-neighbour ring activated (the "buffer region").
The output OccupancyGrid uses three cell classes:
  • Active corridor (seeds + 1-ring buffer) → UNKNOWN (-1).
    Planner traverses here freely (allow_unknown: true).
  • Everything else inside the bounding box → LETHAL (100).
    These are the "wall" cells: 100% intraversable, which is what
    makes Dijkstra fast — search collapses to the corridor.
  • Cells outside the bounding box don't exist (the grid is only
    as wide as the BB).

The bounding box is MONOTONICALLY GROWING within a session — once the
corridor extends to a tile, the global costmap always covers it.
That's how local_mirror_layer's accumulated obstacle cells survive
the master-costmap resizes triggered by static_layer when /map_padded
grows toward a far goal.

map_padder does NOT contribute obstacle data; the only sensor path
into the global costmap is /local_costmap/costmap via the
local_mirror_layer plugin. Map padder is purely geometric: it carves
the corridor and stamps walls around it.
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
        # Half-side of the local-costmap window in meters. Seeds the
        # corridor around the robot so the global covers exactly the
        # ground the local-mirror layer can stamp into. Matches the
        # local costmap's width/2 = 3.0 m in nav2_paramsv2.yaml.
        self.declare_parameter('local_window_radius_m', 3.0)

        self._tile = self.get_parameter('tile_size_m').value
        self._out_res = self.get_parameter('output_resolution').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        self._local_window_radius_m = self.get_parameter(
            'local_window_radius_m').value

        self._tf_buf = Buffer()
        self._tf_lis = TransformListener(self._tf_buf, self)

        self._latest_goal = None
        self._latest_map = None
        self._plan_poses = None
        self._prev_active = None
        # Bounding box of the published canvas accumulates monotonically:
        # once the corridor extends to some tile, the canvas always
        # includes it. Local_mirror_layer's matchSize replays cells into
        # the new master coordinates on every resize, but anything that
        # falls outside the new master gets DROPPED. Monotonic growth
        # ensures we never shrink past previously-stamped obstacle cells
        # — that's how "global remembers past obstacles" survives plan
        # tightening, goal reaching, etc.
        self._bb_min_tx = None
        self._bb_min_ty = None
        self._bb_max_tx = None
        self._bb_max_ty = None
        # Cumulative corridor: once a tile has been part of the active
        # corridor (robot footprint + goal + plan + 1-ring buffer), it
        # stays UNKNOWN forever. New walls are only added in tiles that
        # have never been in the corridor. This prevents map_padder
        # from "eating away" cells behind the robot as the corridor
        # follows the plan forward — old corridor cells never flip
        # back to LETHAL walls.
        self._cumulative_corridor = set()
        # Wall ring (1-tile frontier outside cumulative corridor).
        # Tracked separately for change-detection (skip publish if
        # nothing changed).
        self._prev_wall_ring = set()

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
        # /nav_goal is GoalBender's per-tick output (bent goal when the
        # original goal is behind the robot, pass-through otherwise).
        # Subscribing here gives map_padder a 1 Hz refresh trigger AND
        # ensures the corridor seeds the bent target — without this the
        # planner returns "goal off global costmap" when GoalBender
        # produces a bent target outside the unbent /goal_pose corridor.
        self.create_subscription(PoseStamped, '/nav_goal', self._on_goal, 10)
        # /goal_update carries in-mission goal corrections from
        # gps_handler_node — only the first publish of a leg goes to
        # /goal_pose; subsequent updates flow through /goal_update so
        # the BT's GoalUpdater absorbs them without canceling the
        # FollowPath action. Without subscribing here, map_padder only
        # learns about goal moves when GoalBender re-publishes
        # /nav_goal, which is gated on the BT actively ticking
        # (1 Hz inside the RateController, and skipped while a recovery
        # branch is running). For far GPS waypoints that arrive mid-leg
        # this leaves the corridor stranded at the previous goal until
        # GoalBender next ticks; subscribing directly extends the
        # corridor immediately on the /goal_update publish.
        self.create_subscription(PoseStamped, '/goal_update', self._on_goal, 10)
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
        # /map is used only as a periodic publish trigger now — the map
        # contents are no longer pasted into /map_padded (sensor data
        # may not reach the global costmap; that's the local mirror's
        # job). We still cache it so plan / goal callbacks can publish
        # without waiting for the next /map tick.
        self._latest_map = msg
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
        """Emit a placeholder OccupancyGrid sized to the local-costmap
        window so Nav2's static_layer has something to latch onto
        before the first /map. The bounding box will grow on the first
        real publish; static_layer will resize the master at that
        point (which fires matchSize() on local_mirror_layer — whose
        matchSize override preserves accumulated cells across the
        resize). All cells start UNKNOWN so they don't contaminate the
        global with phantom obstacles.
        """
        res = self._out_res if self._out_res > 0 else 0.10
        side_m = 2.0 * self._local_window_radius_m
        cells = max(1, int(round(side_m / res)))
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
            f'({side_m:.1f}m square @ {res}m res, all UNKNOWN)')

    def _plan_throttle_tick(self):
        if self._pending_plan_publish and self._latest_map:
            self._publish(self._latest_map)
            self._pending_plan_publish = False

    # ── core ─────────────────────────────────────────────────────────

    def _publish(self, msg):
        if msg.info.resolution <= 0:
            return

        tile = self._tile
        res = self._out_res

        # ── 1. seeds ─────────────────────────────────────────────────
        # No SLAM tiles anymore — the global costmap mirrors the local
        # via local_mirror_layer, which is the only obstacle path into
        # the global. Map_padder is purely geometric.
        seeds = set()

        # Robot footprint (the local-costmap window — 3 m radius). At
        # standstill this guarantees the global covers exactly the same
        # ground the local does, so the local_mirror_layer can stamp
        # local's cells into the global with no positional drop-outs.
        rxy = self._robot_xy()
        if rxy:
            rx, ry = rxy
            half = self._local_window_radius_m
            tx0, ty0 = self._tile_of(rx - half, ry - half)
            tx1, ty1 = self._tile_of(rx + half, ry + half)
            for tx in range(tx0, tx1 + 1):
                for ty in range(ty0, ty1 + 1):
                    seeds.add((tx, ty))

        # Goal tile + straight line from robot to goal — extends the
        # global toward distant GPS waypoints.
        if self._latest_goal:
            gx, gy = self._latest_goal
            seeds.add(self._tile_of(gx, gy))
            if rxy is not None:
                seeds |= self._line_tiles(rxy[0], rxy[1], gx, gy)

        # Plan path tiles — keeps the corridor following the latest plan.
        if self._plan_poses:
            for px, py in self._plan_poses:
                seeds.add(self._tile_of(px, py))

        # ── 2. cumulative corridor + retreating wall ─────────────────
        # Per design intent: every seed tile gets a 1-ring of UNKNOWN
        # buffer around it (the corridor). Cells outside the corridor
        # but adjacent to it form a 1-tile-thick LETHAL wall that
        # bounds Dijkstra's search. As the corridor extends (robot
        # moves, plan grows, goal shifts), the wall PUSHES OUTWARD —
        # old wall cells that the corridor reaches become UNKNOWN.
        # Cells that have ever been corridor stay UNKNOWN forever:
        # no eating-away of the map behind the robot.
        new_corridor = self._ring(seeds)
        prev_size = len(self._cumulative_corridor)
        self._cumulative_corridor |= new_corridor

        # 1-tile-thick wall ring around the cumulative corridor.
        wall_ring = self._ring(self._cumulative_corridor) - self._cumulative_corridor

        # Skip publish if nothing extended (cumulative corridor and
        # wall ring are both unchanged).
        if len(self._cumulative_corridor) == prev_size and \
                wall_ring == self._prev_wall_ring:
            return
        self._prev_wall_ring = set(wall_ring)

        all_tiles = self._cumulative_corridor | wall_ring
        if not all_tiles:
            return

        # ── 3. monotonically-growing bounding box ────────────────────
        cur_min_tx = min(t[0] for t in all_tiles)
        cur_max_tx = max(t[0] for t in all_tiles)
        cur_min_ty = min(t[1] for t in all_tiles)
        cur_max_ty = max(t[1] for t in all_tiles)

        if self._bb_min_tx is None:
            self._bb_min_tx, self._bb_max_tx = cur_min_tx, cur_max_tx
            self._bb_min_ty, self._bb_max_ty = cur_min_ty, cur_max_ty
        else:
            self._bb_min_tx = min(self._bb_min_tx, cur_min_tx)
            self._bb_max_tx = max(self._bb_max_tx, cur_max_tx)
            self._bb_min_ty = min(self._bb_min_ty, cur_min_ty)
            self._bb_max_ty = max(self._bb_max_ty, cur_max_ty)

        min_tx, max_tx = self._bb_min_tx, self._bb_max_tx
        min_ty, max_ty = self._bb_min_ty, self._bb_max_ty

        bb_ox = min_tx * tile
        bb_oy = min_ty * tile
        cpt = max(1, round(tile / res))
        bb_w = (max_tx - min_tx + 1) * cpt
        bb_h = (max_ty - min_ty + 1) * cpt

        if bb_w > MAX_GRID_SIDE or bb_h > MAX_GRID_SIDE:
            self.get_logger().warn(
                f'Bounding box {bb_w}x{bb_h} exceeds MAX_GRID_SIDE; clamped')
            bb_w = min(bb_w, MAX_GRID_SIDE)
            bb_h = min(bb_h, MAX_GRID_SIDE)

        # ── 4. tile mask → cell mask ─────────────────────────────────
        # Mask is the CUMULATIVE corridor (every tile ever in a 1-ring
        # of any seed across this session). Cells outside the
        # cumulative corridor but inside the BB end up as LETHAL walls.
        n_tx = max_tx - min_tx + 1
        n_ty = max_ty - min_ty + 1
        tmask = np.zeros((n_ty, n_tx), dtype=np.bool_)
        for tx, ty in self._cumulative_corridor:
            if min_tx <= tx <= max_tx and min_ty <= ty <= max_ty:
                tmask[ty - min_ty, tx - min_tx] = True

        cmask = np.repeat(np.repeat(tmask, cpt, axis=0),
                          cpt, axis=1)[:bb_h, :bb_w]

        # ── 5. build grid: LETHAL walls + UNKNOWN corridor ───────────
        # Default LETHAL (100); cumulative-corridor cells flip to
        # UNKNOWN (-1). The wall (LETHAL) hugs the corridor on its
        # current frontier; old wall cells that the corridor has since
        # reached are inside cumulative_corridor and thus UNKNOWN. The
        # planner sees a tight, monotonically-growing free corridor
        # bounded by intraversable walls — Dijkstra collapses to the
        # corridor, which keeps planning fast.
        grid = self._grid_buf[:bb_h, :bb_w]
        grid.fill(LETHAL)
        grid[cmask] = UNKNOWN

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
        out.data = array.array('b')
        out.data.frombytes(grid.tobytes())
        self._pub.publish(out)

        self.get_logger().info(
            f'corridor={len(self._cumulative_corridor)} wall={len(wall_ring)}  '
            f'{bb_w}x{bb_h} ({bb_w*res:.0f}x{bb_h*res:.0f}m)')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapPadder())
    rclpy.shutdown()
