"""GPS waypoint action server for AutoNav.

Hosts the ``/navigate_to_waypoint`` action (``autonav_interfaces``),
the live magnetometer-less heading EKF, the self-correcting
candidate-goal layers (EWMA smoother → 1/r envelope → moving-away
trip-wire → force-resync), and two coordinate-conversion services.

Architecture (plan_manifest §2.2, §5.12):

  /odometry/filtered (~30 Hz) ─► EKF heartbeat (predict + maybe update
                                  + maybe resync + smoother tick)
  /gps_fix (0.5–10 Hz)        ─► sets the latest measurement; consumed
                                  by the next EKF heartbeat tick
  Action goal active           ─► 1 Hz republisher publishes /goal_pose
                                  (NAV2_GOAL_HZ; faster thrashes A*/Smac)

Two callback groups:

  * ``ReentrantCallbackGroup`` — action server. Cancel can interrupt
    mid-``execute_callback``.
  * ``MutuallyExclusiveCallbackGroup`` — EKF callback, services,
    publish timer. Share a single ``threading.Lock`` over all EKF
    state, gps_history, and candidate-goal state.

Anti-patterns avoided (plan §13):

  * #1 — runs *alongside* navsat_transform_node, doesn't replace it
  * #3 — /goal_pose at exactly 1 Hz
  * #4 — candidate goal recomputed every step, sampled at 1 Hz
  * #5 — bootstrap graduates on ``odom_dist > 5 m AND bs_baseline > 5 m``
  * #6 — magnitude-ratio filter inside ``gps_ekf.closed_form_theta_window``
  * #13 — publisher stopped *before* terminal status returned
  * #14 — candidate anchored on live ``ekf.pos`` not spawn point
  * #15 — moving-away does *not* reset the smoother
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from autonav_interfaces.action import NavigateToWaypoint
from autonav_interfaces.srv import GpsToLocal, LocalToGps

from .gps_ekf import (
    EKF_REJ_STREAK_RESET,
    GpsEkf,
    HistoryEntry,
    bootstrap_theta,
    closed_form_theta_window,
    wrap_pi,
)


# ── Constants (plan_manifest §3 / survey §7). No magic numbers below ──

# Cadence (§3.1)
NAV2_GOAL_HZ: float = 1.0
FEEDBACK_HZ: float = 2.0

# Convergence / arrival (§3.2)
# Tightened from the plan's 1.0 m default to 0.25 m so the action
# terminates promptly once the robot is essentially on the candidate
# goal — earlier behavior was to keep refining indefinitely once the
# robot got within ~0.2 m, blocking the next mission leg.
SUCCESS_RADIUS_M: float = 0.5
"""Default arrival radius for GPS / local / map goals, m. Sits below
the IGVC AutoNav competition threshold so a goal counted as ``SUCCESS``
locally is also a real arrival from the judges' perspective, but is
loose enough to absorb GPS jitter + EKF position noise (empirically
~0.3 m σ outdoors). Earlier tightenings to 0.25 m caused the controller
to occasionally hover near a goal without graduating, blocking the
next mission leg; the prior 1.0 m default was looser than necessary.
Per-goal override available via ``goal_msg.success_radius_m``."""
STOP_REFINE_K: float = 2.0
"""``‖ekf_pos − goal‖ < k · σ_GPS`` ⇒ refinement_locked. k=2."""
STOP_REFINE_SIGMA_GPS_M: float = 0.3
"""σ_GPS used in the stop-refining gate. 0.3 m matches the simulator's
GPS_NOISE_STD calibration; ≈ 0.6 m gate at k=2."""

# Bootstrap (§3.3, §13 #5)
BOOTSTRAP_ODOM_DIST_M: float = 5.0
BOOTSTRAP_BASELINE_M: float = 5.0
BOOTSTRAP_MIN_BASELINE_M: float = 1.5
"""``min_baseline`` passed to ``bootstrap_theta`` while still bootstrapping."""
BOOTSTRAP_WINDOW: int = 100
"""Sliding-anchor window for ``bootstrap_theta``. With encoder yaw
bias on the local EKF, anchoring the closed-form fit on the very
first GPS sample contaminates the estimate with drift accumulated
over the entire history; sliding the anchor to the oldest sample
within the trailing N entries (≈10 s @ 10 Hz GPS) keeps the time
span — and therefore the drift per pair — bounded."""

# Goal republish gating (§3.3)
# ────────────────────────────────────────────────────────────────────
# NOTE: currently UNUSED. Defined as design intent but the value is
# never referenced when deciding whether to republish a goal. The
# republish gate today is GOAL_REPUBLISH_HEARTBEAT_S only. Either
# wire this into _publish_goal (compare theta_shift to this constant
# before triggering republish) or delete it. Kept here so the design
# rationale isn't lost: at 1° the controller saturated max_vel_theta
# on every θ-step (CONTROLLER-CHATTER pattern), so the planned gate
# was 3°.
# ────────────────────────────────────────────────────────────────────
GOAL_REPUBLISH_THETA_DEG: float = 3.0
"""Republish on θ shift > ~3° since the last publish, in addition to
the 1 Hz timer. Raised from 1° based on May-2026 outdoor data: the
controller saturated max_vel_theta on every 1° θ-step, producing a
sharp turn per republish. With ~12° resync magnitudes the previous
threshold made every resync trigger a republish; 3° lets small θ
oscillations near the bias mean settle without commanding turns."""

# ────────────────────────────────────────────────────────────────────
# CONTROLLER-CHATTER-SENSITIVE — GOAL_REPUBLISH_HEARTBEAT_S is the
# active gate; the only thing throttling /goal_pose republishes to
# one every ~5 s post-bootstrap. NAV2's BT triggers a planning pass
# on every fresh /goal_pose, so a faster cadence produces a
# replanning storm and the controller chatters. DO NOT LOWER below
# ~3 s without re-architecting the in-mission update path (today
# /goal_update absorbs small motion between heartbeats).
#
# NOTE: GOAL_REPUBLISH_MIN_DELTA_M is declared but NEVER READ in
# this file (verified). The translational-delta gate the prose
# describes is design intent that wasn't wired in. Either implement
# it (compare distance to last_published_goal_map in _publish_goal)
# or delete the constant. Documented here so the original tuning
# rationale (1.5 m matches controller path-tolerance; 0.30 m was
# the value that reproduced chatter) isn't lost.
# ────────────────────────────────────────────────────────────────────
GOAL_REPUBLISH_MIN_DELTA_M: float = 1.5
GOAL_REPUBLISH_HEARTBEAT_S: float = 5.0

# ────────────────────────────────────────────────────────────────────
# CONTROLLER-CHATTER-SENSITIVE — DO NOT lower without re-architecting
# the in-mission update path. /goal_pose republishes go through the
# NavigateToPose action and force bt_navigator to re-engage, which
# looks identical to a fresh goal and triggers a full replan. 10 s
# (the earlier value) produced periodic replanning the controller
# couldn't fully absorb; 60 s lets /goal_update absorb normal
# in-mission motion through the GoalUpdater decorator and saves
# the heavyweight /goal_pose for true lifecycle loss.
# ────────────────────────────────────────────────────────────────────
GOAL_POSE_HEARTBEAT_S: float = 60.0

# ────────────────────────────────────────────────────────────────────
# CONTROLLER-CHATTER-SENSITIVE — heading resync cadence. Each resync
# event corrects accumulated EKF yaw bias (which is map↔odom-adjacent
# in that the bias originates upstream of slam_toolbox's correction
# loop), but the symptom on the robot is a sharp controller turn per
# resync. Firing too often (10°/3 s) reproduced the start-stop
# pattern as a sequence of replans + turns the controller couldn't
# fade between. 15°/5 s lets θ oscillate around the bias mean
# without triggering, while still catching real drift (exceeds 15°
# within 30 s). DO NOT lower threshold or shorten cooldown without
# first verifying the encoder calibration / imu_cov_inflator chain
# is still constraining yaw bias upstream.
# ────────────────────────────────────────────────────────────────────
HEADING_RESYNC_THRESHOLD_DEG: float = 15.0
HEADING_RESYNC_COOLDOWN_S: float = 5.0
HEADING_RESYNC_WINDOW: int = 100
HEADING_RESYNC_MIN_BASELINE_M: float = 2.0
HEADING_RESYNC_VAR_DEG: float = 5.0

# Health monitor thresholds. Surface DEGRADED / FAIL on
# /gps_waypoint/health so the operator catches regressions
# (frozen pose, runaway heading drift) before they wreck a
# mission. Anchored to outdoor-mission empirical numbers:
# a known-good run produced theta drift well under 0.1°/s,
# the broken run produced 0.3-2°/s.
HEALTH_WINDOW_S: float = 10.0
HEALTH_DEGRADED_THETA_DEG: float = 5.0   # over HEALTH_WINDOW_S
HEALTH_FAIL_THETA_DEG: float = 15.0      # over HEALTH_WINDOW_S
HEALTH_ODOM_STALE_S: float = 1.0
HEALTH_FAIL_BOOTSTRAP_AFTER_MOTION_M: float = 5.0

# Periodic unconditional heading refit (sim parity). Every
# PERIODIC_REFIT_PERIOD_S, refit θ from the full window if it
# disagrees with the EKF's current θ by more than the threshold.
# Catches small persistent biases (5-10°) that fall below the
# moving-away (1 m / 3 s) and divergence (5 m) thresholds yet
# still cause meters of cross-track error over a long goal.
PERIODIC_REFIT_PERIOD_S: float = 3.0
PERIODIC_REFIT_THRESHOLD_DEG: float = 10.0
PERIODIC_REFIT_MIN_BASELINE_M: float = 2.0
PERIODIC_REFIT_VAR_DEG: float = 5.0

# Force resync — fired by moving-away (§3.5, §10.6.4)
HEADING_FORCE_RESYNC_WINDOW: int = 500
HEADING_FORCE_RESYNC_MIN_BASELINE_M: float = 3.0
HEADING_FORCE_RESYNC_DIFF_DEG: float = 20.0
HEADING_FORCE_RESYNC_VAR_DEG: float = 10.0

# ────────────────────────────────────────────────────────────────────
# CONTROLLER-CHATTER-SENSITIVE — candidate-goal smoother. UPSTREAM
# defense against GPS jitter → goal-tick → replan-storm chatter.
# DO NOT TUNE without watching /gps_waypoint/debug telemetry across
# at least one full mission.
# CANDIDATE_SMOOTH_ALPHA=0.15: ~6-tick (≈0.6 s @ 10 Hz) ease-in.
# CANDIDATE_SNAP_M=10: anything below is EWMA-eased; anything above
#   is a legitimate large correction that bypasses the smoother.
#   Was 5 m and made every heading-resync bypass the EWMA, causing
#   the goal to jump every tick.
# CANDIDATE_ENV_FLOOR_M=1.0: noise floor of the 1/r envelope filter.
#   Was 0.4 and dropped legitimate corrections during bootstrap.
# ────────────────────────────────────────────────────────────────────
# Candidate-goal smoother (§3.5)
CANDIDATE_SMOOTH_ALPHA: float = 0.15
CANDIDATE_SNAP_M: float = 10.0
CANDIDATE_ENV_GAIN_M: float = 0.5
CANDIDATE_ENV_FLOOR_M: float = 1.0
CANDIDATE_ENV_REJECT_K: float = 4.0
CANDIDATE_ENV_MIN_R_M: float = 3.0

# Moving-away detector (§3.5, §10.6.3)
MOVING_AWAY_WINDOW_S: float = 3.0
MOVING_AWAY_THRESHOLD_M: float = 1.0
MOVING_AWAY_ENV_SUSPEND_S: float = 4.0
MOVING_AWAY_MIN_HISTORY_TICKS: int = 8
"""Reduced from 25. Real-robot GPS publishes at ~1-2 Hz; the prior
25-tick floor blacked out the detector for 12-25 s, longer than the
window in which the robot would drive off-target. 8 ticks ≈ 8 s
@ 1 Hz, 4 s @ 2 Hz — fast enough to catch early divergence."""
# Require the oldest sample in the moving-away history deque to span at
# least this fraction of MOVING_AWAY_WINDOW_S before we trust the
# delta-distance comparison. Guards against firing the detector with a
# half-filled window right after _dist_history is cleared.
MOVING_AWAY_WINDOW_COVERAGE: float = 0.6
"""Loosened from 0.8 for the same low-GPS-rate reason as
MOVING_AWAY_MIN_HISTORY_TICKS — 0.8 demanded 2.4 s of span, which
at 1 Hz GPS means at least 3 samples must already be present."""

# Local-vs-world divergence detector — second trip wire next to
# moving-away. Moving-away catches *radial* drift (raw GPS distance
# growing). It misses *tangential* drift, where the robot moves
# perpendicular to the goal radial: raw GPS distance barely changes
# while the EKF-believed distance can collapse rapidly because θ is
# wrong and the predict step rotates odom motion into a fictitious
# "toward goal" direction. This detector compares cumulative progress
# in EKF-frame distance vs raw-GPS-frame distance since goal
# acceptance — when local progress runs ahead of world progress by
# more than ``LOCAL_VS_WORLD_DIVERGENCE_M`` and we've accumulated
# at least ``LOCAL_VS_WORLD_MIN_LOCAL_PROGRESS_M`` of EKF progress,
# θ is wrong and we force a heading resync.
LOCAL_VS_WORLD_DIVERGENCE_M: float = 5.0
LOCAL_VS_WORLD_MIN_LOCAL_PROGRESS_M: float = 5.0
LOCAL_VS_WORLD_COOLDOWN_S: float = 5.0

# History trim (§7 GPS_HISTORY_LEN)
GPS_HISTORY_LEN: int = 400

# Liveness
GPS_STALE_TIMEOUT_S: float = 5.0
TF_TIMEOUT_S: float = 0.5
"""``map → odom`` lookup timeout. Skip the publish on TF failure rather
than emit a stale pose (manifest §5.6 "TF safety")."""

EARTH_R_M: float = 6_371_000.0
"""Earth radius (m), used in the equirectangular lat/lon → meters
linearization around the datum."""

# Antenna lever-arm. The GPS antenna does not sit at base_link; the
# URDF puts it ~38 cm behind and ~56 cm above on Bowser. Without
# subtracting that offset before the EKF update, every GPS sample
# is biased by R(yaw_world) · antenna_offset_in_baselink — a
# heading-correlated bias that locks the EKF onto a fixed wrong
# point. We pull the offset live from TF on the first GPS sample
# (so URDF edits are picked up automatically).
GPS_LINK_FRAME: str = "gps_footprint"
BASE_LINK_FRAME: str = "base_link"


# ── Helpers ─────────────────────────────────────────────────────────

def latlon_to_local(
    lat_deg: float,
    lon_deg: float,
    datum_lat_deg: float,
    datum_lon_deg: float,
) -> Tuple[float, float]:
    """Equirectangular linearization around a datum. Same as
    ``GPSHandler.calculate_distance`` but without the file IO. Returns
    ``(x_east_m, y_north_m)``.
    """
    ref_lat_rad = math.radians(datum_lat_deg)
    dlat = math.radians(lat_deg - datum_lat_deg)
    dlon = math.radians(lon_deg - datum_lon_deg)
    y = dlat * EARTH_R_M
    x = dlon * EARTH_R_M * math.cos(ref_lat_rad)
    return x, y


def local_to_latlon(
    x_m: float,
    y_m: float,
    datum_lat_deg: float,
    datum_lon_deg: float,
) -> Tuple[float, float]:
    """Inverse of :func:`latlon_to_local`."""
    ref_lat_rad = math.radians(datum_lat_deg)
    lat = datum_lat_deg + math.degrees(y_m / EARTH_R_M)
    if abs(math.cos(ref_lat_rad)) < 1e-9:
        lon = datum_lon_deg
    else:
        lon = datum_lon_deg + math.degrees(x_m / (EARTH_R_M * math.cos(ref_lat_rad)))
    return lat, lon


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Z-axis yaw from a unit quaternion. Identity ⇒ 0.0."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


# ── Active-goal bookkeeping ─────────────────────────────────────────

@dataclass
class _ActiveGoal:
    """All the per-goal state the publisher and feedback loop need."""

    handle: ServerGoalHandle
    goal_type: int  # GOAL_TYPE_GPS or GOAL_TYPE_LOCAL
    frame_id: str
    success_radius_m: float
    final_yaw: Optional[float]  # None ⇒ auto-yaw toward goal
    # GPS goals carry a (lat, lon); local goals leave it None.
    goal_lat_lon: Optional[Tuple[float, float]]
    # Native-frame XY of the goal:
    #   GPS goal           — world (lat/lon-projected, datum origin) frame
    #   LOCAL "map" goal   — map frame (publish as-is)
    #   LOCAL "odom" goal  — REJECTED at acceptance (see L1 scope decision)
    # Field name kept as ``goal_world_xy`` to minimize churn; comment
    # documents the reinterpretation for local goals.
    goal_world_xy: Tuple[float, float]
    # For LOCAL goals, the input frame ("map"). Always None for GPS goals.
    local_input_frame: Optional[str]
    # Goal start timestamp expressed in node-clock seconds (the same source
    # used by ``self._now_s()``). Despite the historical ``_wall`` suffix in
    # earlier revisions, this is ROS time — preferred for sim-time / bag
    # replay compatibility — not ``time.time()`` wall-clock seconds.
    started_ros_s: float
    # Diagnostics accumulated for the Result.
    peak_theta_std_deg: float = 0.0
    distance_traveled_m: float = 0.0
    last_pos_xy: Optional[Tuple[float, float]] = None
    last_published_goal_world: Optional[Tuple[float, float]] = None
    last_published_goal_map: Optional[Tuple[float, float]] = None
    last_published_theta: Optional[float] = None
    # Wall-time-ish timestamp of the last successful /goal_pose publish.
    # Drives the heartbeat gate of the goal-republish throttle so a
    # stalled NAV2 still picks up state every GOAL_REPUBLISH_HEARTBEAT_S.
    last_published_t_s: Optional[float] = None
    # Wall-time-ish timestamp of the last /goal_pose (NavigateToPose
    # action) publish — distinct from last_published_t_s above, which
    # covers /goal_update too. In-mission corrections go to
    # /goal_update so the BT's GoalUpdater can absorb them without
    # canceling FollowPath; we still kick a /goal_pose every
    # GOAL_POSE_HEARTBEAT_S as a safety belt against bt_navigator
    # losing its action handle.
    last_goal_pose_t_s: float = 0.0
    # Local-vs-world divergence detector baselines. Lazy-initialized
    # on the first odom tick that has both a valid EKF position and a
    # valid raw GPS sample — they can't be set at goal acceptance
    # because GPS may not have arrived yet. Cumulative progress is
    # measured against these two baselines until the goal terminates
    # or a divergence event resets them.
    local_d_start: Optional[float] = None
    world_d_start: Optional[float] = None
    # Preempt-with-cancel signaling (§5.14, §13 anti-patterns).
    #   ``preempt_requested`` is set by a *newer* goal's execute_callback
    #   when it wants to take over. The prior goal's own execute_callback
    #   loop polls this flag in addition to ``handle.is_cancel_requested``
    #   and routes to the terminal path with ``STATUS_PREEMPTED``. We
    #   cannot call ``handle.canceled()`` / ``handle.abort()`` from a
    #   foreign callback — only the goal's owning execute_callback may
    #   transition its goal handle. Doing otherwise corrupts the
    #   rclpy_action state machine.
    #   ``preempt_done`` is set by ``_terminate`` once the slot has been
    #   cleared, so the new goal's wait stops promptly without polling.
    preempt_requested: bool = False
    preempt_done: Optional[threading.Event] = None

    def __post_init__(self) -> None:
        if self.preempt_done is None:
            self.preempt_done = threading.Event()


# ── Node ────────────────────────────────────────────────────────────


class GpsHandlerNode(Node):
    """Self-correcting GPS waypoint handler — see module docstring."""

    def __init__(self) -> None:
        super().__init__("gps_handler_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("success_radius_m", SUCCESS_RADIUS_M)
        self.declare_parameter("nav2_goal_hz", NAV2_GOAL_HZ)
        self.declare_parameter("feedback_hz", FEEDBACK_HZ)
        self.declare_parameter("gps_stale_timeout_s", GPS_STALE_TIMEOUT_S)
        self.declare_parameter("tf_timeout_s", TF_TIMEOUT_S)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")

        self._success_radius_default: float = float(
            self.get_parameter("success_radius_m").value
        )
        self._nav2_goal_hz: float = float(
            self.get_parameter("nav2_goal_hz").value
        )
        self._feedback_hz: float = float(
            self.get_parameter("feedback_hz").value
        )
        self._gps_stale_timeout_s: float = float(
            self.get_parameter("gps_stale_timeout_s").value
        )
        self._tf_timeout_s: float = float(
            self.get_parameter("tf_timeout_s").value
        )
        self._map_frame: str = str(self.get_parameter("map_frame").value)
        self._odom_frame: str = str(self.get_parameter("odom_frame").value)

        # ── Threading & callback groups ─────────────────────────────
        self._lock = threading.Lock()
        self._action_cbg = ReentrantCallbackGroup()
        self._estimator_cbg = MutuallyExclusiveCallbackGroup()

        # ── EKF + bootstrap state ───────────────────────────────────
        self._ekf: GpsEkf = GpsEkf()
        # The original design had a "bootstrap_done" graduation milestone
        # — pre-bootstrap fixes hard-reset θ on every sample, post-bootstrap
        # the EKF ran predict/update normally. That two-state machine
        # caused field deadlocks: NAV2 wouldn't translate the robot
        # without a goal, the bootstrap couldn't fit without translation,
        # robot sat stuck. The unified design (matching the simulator at
        # Claude-Sandbox/GPS-Waypoint-Simulation) is to run the EKF
        # continuously from tick 1 with high initial θ variance — the
        # first closed-form fit's Kalman gain is ≈1.0 anyway because
        # P[2,2] starts at π², so a single soft update behaves like the
        # old hard reset. Later updates have small gain and the candidate
        # converges to the true GPS goal as the robot moves.
        #
        # Setting True from init means: the forced-reset bootstrap branch
        # in _gps_callback is unreachable (kept for documentation/safety);
        # _periodic_heading_refit and _maybe_resync_heading always use
        # the Kalman update_theta_measurement path; the 5 s /goal_pose
        # rate limit is active from the first goal.
        self._bootstrap_done: bool = True
        self._gps_history: Deque[HistoryEntry] = deque(maxlen=GPS_HISTORY_LEN)

        # Datum (lat/lon of the very first valid GPS fix).
        self._datum_lat: Optional[float] = None
        self._datum_lon: Optional[float] = None

        # Latest GPS in world meters and timestamp.
        self._last_gps_xy: Optional[Tuple[float, float]] = None
        self._last_gps_stamp_s: Optional[float] = None
        self._gps_pending: bool = False  # set by /gps_fix, cleared by EKF tick

        # Latest odom snapshot, for predict deltas + bootstrap pairs.
        self._last_odom_xy: Optional[Tuple[float, float]] = None
        # Robot's odom-frame yaw, extracted from each odom message. We
        # need it (combined with self._ekf.theta) to know the world
        # heading for the antenna lever-arm correction in _gps_callback.
        self._last_odom_yaw: float = 0.0
        # Cached base_link → gps_footprint translation. Looked up lazily
        # on the first GPS callback (TF tree may not be live at __init__).
        self._gps_link_offset_xy: Optional[Tuple[float, float]] = None
        self._last_odom_stamp_s: Optional[float] = None
        self._odom_distance_m: float = 0.0

        # Heading-resync cooldown (uses node clock seconds).
        self._heading_resync_until_s: float = 0.0
        self._heading_resync_count: int = 0

        # Candidate-goal smoother + envelope state.
        self._smoothed_candidate: Optional[Tuple[float, float]] = None
        self._envelope_suspended_until_s: float = 0.0
        # Hard cap as defense-in-depth. Time-window trim in
        # ``_update_moving_away`` keeps this near MOVING_AWAY_WINDOW_S
        # × tick rate (~30 Hz × 3 s ≈ 90 entries). The cap protects
        # against pathological clock-skew or paused-trim scenarios from
        # unbounded growth over a long-running session.
        self._dist_history: Deque[Tuple[float, float]] = deque(maxlen=512)
        self._moving_away_event_count: int = 0
        self._cand_reject_count: int = 0
        # Local-vs-world divergence detector cooldown timestamp.
        self._divergence_cooldown_until_s: float = 0.0

        # Active-goal slot.
        self._active: Optional[_ActiveGoal] = None
        # Anti-pattern §13 #13: when a cancel/preempt is acknowledged,
        # stop the 1 Hz /goal_pose republisher *immediately* (i.e. on
        # the very next tick) so NAV2's BT can't see one more stale
        # goal between cancel-ack and the execute loop's next
        # is_cancel_requested poll. ``_terminate`` finally clears
        # ``_active`` itself, but until then the publisher checks this
        # flag and early-returns.
        self._publisher_disabled: bool = False

        # ── TF ─────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self
        )

        # ── Subscriptions ───────────────────────────────────────────
        # /gps_fix: most hardware GPS drivers publish BEST_EFFORT — use
        # the canonical sensor-data profile so we don't silently drop
        # every message via a reliability mismatch. The publisher
        # (gps_publisher.cpp) is configured with the same SensorDataQoS
        # so DDS matches deterministically across pub respawns.
        #
        # The /gps_fix sub lives in its OWN ReentrantCallbackGroup so
        # high-rate odom callbacks in ``_estimator_cbg`` cannot starve
        # the GPS callback under load. Shared state is still protected
        # by ``self._lock`` (snapshot/short-critical-section pattern).
        self._gps_cbg = ReentrantCallbackGroup()
        self.create_subscription(
            NavSatFix,
            "/gps_fix",
            self._gps_callback,
            qos_profile_sensor_data,
            callback_group=self._gps_cbg,
        )
        # Local-frame fused odom from robot_localization's ekf_filter_node.
        #
        # The slam.launch.py configures ekf_local with the remap
        #   ('odometry/filtered', 'local_ekf/odom')
        # so the local EKF publishes to ``/local_ekf/odom``, NOT
        # ``/odometry/filtered``. The global EKF (``ekf_global``) is the
        # node that *would* normally publish ``/odometry/filtered``, but
        # it's commented out of the LaunchDescription. Subscribing to
        # ``/odometry/filtered`` therefore receives nothing — the bug
        # that kept the gps_handler EKF starved of odom updates and
        # blocked θ-bootstrap convergence the entire time the system
        # was deployed. Subscribe to the actual published topic name.
        #
        # robot_localization's ekf_filter_node publishes RELIABLE
        # depth=1, so match that exactly.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Odometry,
            "/local_ekf/odom",
            self._odom_callback,
            odom_qos,
            callback_group=self._estimator_cbg,
        )

        # ── Publishers ──────────────────────────────────────────────
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._goal_pub = self.create_publisher(
            PoseStamped, "/goal_pose", pub_qos
        )
        # In-mission corrections (1 Hz smoothed candidate) go here.
        # Nav2's bt_nav.xml wraps the planner in a <GoalUpdater> that
        # consumes /goal_update and rewrites the BT's {live_goal} in
        # place — FollowPath is NOT canceled, so the controller keeps
        # emitting cmd_vel uninterrupted across goal refreshes.
        self._goal_update_pub = self.create_publisher(
            PoseStamped, "/goal_update", pub_qos
        )
        diag_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._heading_pub = self.create_publisher(
            Float64, "/gps_waypoint/heading_offset", diag_qos
        )
        self._heading_std_pub = self.create_publisher(
            Float64, "/gps_waypoint/heading_offset_std_deg", diag_qos
        )
        self._debug_pub = self.create_publisher(
            String, "/gps_waypoint/debug", diag_qos
        )
        self._marker_pub = self.create_publisher(
            Marker, "/gps_waypoint/candidate_marker", diag_qos
        )
        # /gps_waypoint/health surfaces the kind of regressions that
        # the May 2026 outdoor mission run exposed: stale /local_ekf/
        # odom feeding a phantom robot pose into goal placement, and
        # heading_offset drift indicating an upstream yaw bias. The
        # GUI / operator can render this as a single status badge and
        # see DEGRADED / FAIL before a real mission goes sideways.
        self._health_pub = self.create_publisher(
            String, "/gps_waypoint/health", diag_qos
        )

        # ── Timers ──────────────────────────────────────────────────
        # /goal_pose republisher is gated on an active goal.
        self._publisher_timer = self.create_timer(
            1.0 / max(self._nav2_goal_hz, 0.1),
            self._publisher_tick,
            callback_group=self._estimator_cbg,
        )
        # Diagnostic publisher always-on at 5 Hz — cheap.
        self._diag_timer = self.create_timer(
            0.2,
            self._diag_tick,
            callback_group=self._estimator_cbg,
        )
        # Health publisher at 1 Hz — emits one /gps_waypoint/health
        # String per second so the GUI can render a status badge.
        self._health_timer = self.create_timer(
            1.0,
            self._health_tick,
            callback_group=self._estimator_cbg,
        )
        # Rolling theta history for drift-rate computation. Stores
        # (wall_time_s, theta_rad) tuples; capped at HEALTH_WINDOW_S
        # by the health tick so memory stays bounded.
        self._theta_history: deque = deque()

        # ── Action server ───────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            NavigateToWaypoint,
            "/navigate_to_waypoint",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cbg,
        )

        # ── Services ────────────────────────────────────────────────
        self.create_service(
            GpsToLocal,
            "/gps_waypoint/gps_to_local",
            self._srv_gps_to_local,
            callback_group=self._estimator_cbg,
        )
        self.create_service(
            LocalToGps,
            "/gps_waypoint/local_to_gps",
            self._srv_local_to_gps,
            callback_group=self._estimator_cbg,
        )

        self.get_logger().info(
            f"gps_handler_node up — /goal_pose @ {self._nav2_goal_hz:.1f} Hz, "
            f"feedback @ {self._feedback_hz:.1f} Hz, success_radius "
            f"= {self._success_radius_default:.2f} m"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup_robot_in_map(self) -> Optional[Tuple[float, float]]:
        """Return the robot's (x, y) in the map frame via TF.

        Used by the LOCAL/map-frame arrival check so distance is
        computed in the goal's native frame. Returns ``None`` on TF
        failure — caller decides on a fallback. Cheap; called once per
        feedback tick (~2 Hz).
        """
        try:
            tf_map_base = self._tf_buffer.lookup_transform(
                self._map_frame,
                "base_link",
                Time(),
                rclpy.duration.Duration(seconds=self._tf_timeout_s),
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ):
            return None
        return (
            float(tf_map_base.transform.translation.x),
            float(tf_map_base.transform.translation.y),
        )

    # ── Sensor callbacks ───────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """EKF heartbeat. Predict on every odom message; if a fresh GPS
        fix is queued, run update + maybe-resync; always tick the
        candidate-goal smoother + moving-away detector. Always-on.
        """
        ox = float(msg.pose.pose.position.x)
        oy = float(msg.pose.pose.position.y)
        # Track odom-frame yaw too — used by the antenna lever-arm
        # correction in _gps_callback (yaw_world = odom_yaw + ekf.theta).
        oq = msg.pose.pose.orientation
        odom_yaw = quat_to_yaw(
            float(oq.x), float(oq.y), float(oq.z), float(oq.w)
        )
        stamp_s = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )
        if stamp_s == 0.0:  # untimestamped messages — fall back to wall clock
            stamp_s = self._now_s()

        with self._lock:
            if self._last_odom_xy is None:
                self._last_odom_xy = (ox, oy)
                self._last_odom_stamp_s = stamp_s
                return
            last_stamp = self._last_odom_stamp_s or stamp_s
            dt = stamp_s - last_stamp
            if dt < 0.0:
                # Clock jumped backward (sim time stepped back, bag-replay
                # seek-rewind, or a new clock source). Don't integrate
                # negative time — reset the predict reference and any
                # absolute-deadline state anchored to the old clock.
                self.get_logger().warn(
                    f"Backward clock jump detected: dt={dt:.3f}s. "
                    f"Resetting EKF predict reference."
                )
                self._last_odom_xy = (ox, oy)
                self._last_odom_stamp_s = stamp_s
                self._dist_history.clear()
                self._heading_resync_until_s = 0.0
                self._envelope_suspended_until_s = 0.0
                return
            if dt > 5.0:
                # Long gap (paused executor, stalled topic, big sim step).
                # Cap so we don't integrate a huge step into the EKF.
                self.get_logger().warn(
                    f"Large dt={dt:.3f}s — capping to 5s."
                )
                dt = 5.0
            if dt < 1e-6:
                # Same timestamp as the previous tick — predict is a
                # no-op; skip F·P·Fᵀ idle work and keep the cached
                # reference unchanged.
                return
            dxo = ox - self._last_odom_xy[0]
            dyo = oy - self._last_odom_xy[1]
            self._last_odom_xy = (ox, oy)
            self._last_odom_yaw = odom_yaw
            self._last_odom_stamp_s = stamp_s
            self._odom_distance_m += math.hypot(dxo, dyo)

            # ── Predict ────────────────────────────────────────────
            self._ekf.predict(dxo, dyo, dt)

            # ── Update on pending GPS ──────────────────────────────
            if self._gps_pending and self._last_gps_xy is not None:
                self._gps_pending = False
                zx, zy = self._last_gps_xy

                if not self._bootstrap_done:
                    # Bootstrap: forcibly reseed θ on every fix while
                    # we're still under the graduation threshold. Pass
                    # the deque directly — ``bootstrap_theta`` iterates
                    # in a single pass without copying.
                    #
                    # ``bootstrap_theta`` accepts an optional ``window``
                    # arg for sliding-anchor mode (mirrors the sim).
                    # Tested at window=100 and it regressed convergence
                    # in the heavy-encoder-bias regime — bootstrap
                    # graduates before the window can clip, and the
                    # long-tail slow-bootstrap agents benefit from the
                    # full-history baseline. Default (window=None,
                    # anchor-on-first) is what the deployed system uses.
                    bs_theta, baseline = bootstrap_theta(
                        self._gps_history,
                        min_baseline=BOOTSTRAP_MIN_BASELINE_M,
                    )
                    if bs_theta is not None:
                        # σ_θ ~ σ_GPS / baseline (manifest §5.4)
                        sigma = max(
                            math.radians(3.0),
                            STOP_REFINE_SIGMA_GPS_M / max(baseline, 0.5),
                        )
                        self._ekf.reset_theta(bs_theta, theta_var=sigma ** 2)
                    # Don't gate during bootstrap.
                    self._ekf.update(zx, zy, gate_chi2=1.0e9)

                    # Graduate when both odom_dist > 5 m AND baseline > 5 m.
                    if (
                        bs_theta is not None
                        and baseline > BOOTSTRAP_BASELINE_M
                        and self._odom_distance_m > BOOTSTRAP_ODOM_DIST_M
                    ):
                        self._bootstrap_done = True
                        self.get_logger().info(
                            f"EKF bootstrap graduated: "
                            f"θ = {math.degrees(self._ekf.theta):+.2f}°, "
                            f"baseline = {baseline:.2f} m, "
                            f"odom = {self._odom_distance_m:.2f} m"
                        )
                else:
                    # Lock-in recovery: if rejection streak hit, force
                    # accept this sample.
                    if self._ekf.consecutive_rejects >= EKF_REJ_STREAK_RESET:
                        self._ekf.force_accept_next()
                        self._ekf.update(zx, zy, gate_chi2=1.0e9)
                    else:
                        self._ekf.update(zx, zy)
                    self._maybe_resync_heading()

            # ── Self-correction layers ─────────────────────────────
            self._update_candidate_smoother()
            self._update_moving_away()
            self._update_local_world_divergence()

    def _gps_callback(self, msg: NavSatFix) -> None:
        """Convert the fix to local meters around the datum, append to
        history, and queue it for the next EKF heartbeat tick."""
        if msg.status.status < 0:  # NavSatStatus.STATUS_NO_FIX = -1
            return
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not math.isfinite(lat) or not math.isfinite(lon):
            return
        stamp_s = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )
        if stamp_s == 0.0:
            stamp_s = self._now_s()

        with self._lock:
            if self._datum_lat is None:
                # First valid fix is the datum. The plan's open question
                # §6 says navsat_transform_node should anchor on the
                # same first-fix; field calibration step.
                self._datum_lat = lat
                self._datum_lon = lon
                self.get_logger().info(
                    f"datum set: lat={lat:.7f}, lon={lon:.7f}"
                )
            zx, zy = latlon_to_local(
                lat, lon, self._datum_lat, self._datum_lon
            )

            # ── Antenna lever-arm correction ──────────────────────
            # /gps_fix is the antenna's world position. The EKF tracks
            # base_link's world position, so we have to subtract
            # R(yaw_world) · antenna_offset_baselink from the raw fix
            # before fusing. yaw_world = odom_yaw + ekf.theta (the
            # rotation between odom and world).
            if self._gps_link_offset_xy is None:
                try:
                    tf = self._tf_buffer.lookup_transform(
                        BASE_LINK_FRAME,
                        GPS_LINK_FRAME,
                        Time(),
                        rclpy.duration.Duration(seconds=0.5),
                    )
                    ax = float(tf.transform.translation.x)
                    ay = float(tf.transform.translation.y)
                    self._gps_link_offset_xy = (ax, ay)
                    self.get_logger().info(
                        f"GPS antenna offset (base_link→{GPS_LINK_FRAME}): "
                        f"x={ax:+.3f}m y={ay:+.3f}m"
                    )
                except (
                    LookupException,
                    ConnectivityException,
                    ExtrapolationException,
                ):
                    # TF not yet ready — defer correction until next fix.
                    pass

            if self._gps_link_offset_xy is not None and (
                abs(self._gps_link_offset_xy[0]) > 1e-3
                or abs(self._gps_link_offset_xy[1]) > 1e-3
            ):
                ax, ay = self._gps_link_offset_xy
                yaw_world = self._last_odom_yaw + self._ekf.theta
                c = math.cos(yaw_world)
                s = math.sin(yaw_world)
                dx_world = c * ax - s * ay
                dy_world = s * ax + c * ay
                zx -= dx_world
                zy -= dy_world

            odom_xy = self._last_odom_xy or (0.0, 0.0)
            self._gps_history.append((stamp_s, (zx, zy), odom_xy))
            self._last_gps_xy = (zx, zy)
            self._last_gps_stamp_s = stamp_s
            self._gps_pending = True

    # ── Heading resync (§5.5 / sim 1681-1715) ──────────────────────

    def _maybe_resync_heading(self) -> None:
        """Lock held. Standard cooldown-gated resync against a 100-sample
        sliding window. ``_force_heading_resync`` is the moving-away-
        triggered, wider-window, no-cooldown variant."""
        if self._now_s() < self._heading_resync_until_s:
            return
        # Pass the deque directly — ``closed_form_theta_window`` uses
        # ``itertools.islice`` to window without materializing a list.
        bs_theta, baseline = closed_form_theta_window(
            self._gps_history,
            HEADING_RESYNC_WINDOW,
            min_baseline=HEADING_RESYNC_MIN_BASELINE_M,
        )
        if bs_theta is None or baseline < HEADING_RESYNC_MIN_BASELINE_M:
            return
        diff = wrap_pi(bs_theta - self._ekf.theta)
        if abs(diff) > math.radians(HEADING_RESYNC_THRESHOLD_DEG):
            # After bootstrap, use a confidence-weighted Kalman update
            # rather than a hard snap-replace. As P[2,2] shrinks the
            # gain falls, so a converged θ becomes increasingly
            # resistant to single noisy fits and the candidate goal
            # in map frame actually converges instead of swinging on
            # every resync. The bootstrap path retains reset_theta
            # because there is no accumulated confidence to weigh
            # against during cold start.
            if self._bootstrap_done:
                self._ekf.update_theta_measurement(
                    bs_theta,
                    theta_meas_std=math.radians(HEADING_RESYNC_VAR_DEG),
                )
            else:
                self._ekf.reset_theta(
                    bs_theta,
                    theta_var=math.radians(HEADING_RESYNC_VAR_DEG) ** 2,
                )
            self._heading_resync_until_s = (
                self._now_s() + HEADING_RESYNC_COOLDOWN_S
            )
            self._heading_resync_count += 1

    def _periodic_heading_refit(self) -> None:
        """Timer callback: unconditional periodic θ refit, every
        PERIODIC_REFIT_PERIOD_S. Sim parity — catches small persistent
        biases below the moving-away/divergence thresholds before they
        translate into meters of cross-track drift. Guarded by an
        active goal and bootstrap completion. Distinct from
        ``_maybe_resync_heading`` (which is GPS-callback-driven and
        cooldown-gated) and ``_force_heading_resync`` (which is
        detector-driven, wider window, larger threshold).
        """
        with self._lock:
            if self._active is None:
                return
            if not self._bootstrap_done:
                return
            bs_theta, baseline = closed_form_theta_window(
                self._gps_history,
                HEADING_RESYNC_WINDOW,
                min_baseline=PERIODIC_REFIT_MIN_BASELINE_M,
            )
            if bs_theta is None or baseline < PERIODIC_REFIT_MIN_BASELINE_M:
                return
            diff = wrap_pi(bs_theta - self._ekf.theta)
            if abs(diff) <= math.radians(PERIODIC_REFIT_THRESHOLD_DEG):
                return
            # Same A/B as _maybe_resync_heading: Kalman update post-
            # bootstrap so periodic refits respect accumulated
            # confidence; bootstrap path keeps the hard reset.
            if self._bootstrap_done:
                self._ekf.update_theta_measurement(
                    bs_theta,
                    theta_meas_std=math.radians(PERIODIC_REFIT_VAR_DEG),
                )
            else:
                self._ekf.reset_theta(
                    bs_theta,
                    theta_var=math.radians(PERIODIC_REFIT_VAR_DEG) ** 2,
                )
            self._heading_resync_count += 1
            self.get_logger().warn(
                f"periodic heading refit: θ_old={math.degrees(self._ekf.theta - diff):+.2f}°, "
                f"θ_new={math.degrees(bs_theta):+.2f}°, "
                f"diff={math.degrees(diff):+.2f}°, baseline={baseline:.2f}m"
            )

    def _force_heading_resync(self) -> bool:
        """Lock held. Wide window (500 samples), 3 m floor, 20° diff —
        no cooldown. Sim 1717-1747."""
        # Pass the deque directly (no list copy) — see
        # ``_maybe_resync_heading`` for the rationale.
        bs_theta, baseline = closed_form_theta_window(
            self._gps_history,
            HEADING_FORCE_RESYNC_WINDOW,
            min_baseline=HEADING_FORCE_RESYNC_MIN_BASELINE_M,
        )
        if bs_theta is None:
            return False
        diff = wrap_pi(bs_theta - self._ekf.theta)
        if abs(diff) < math.radians(HEADING_FORCE_RESYNC_DIFF_DEG):
            return False
        self._ekf.reset_theta(
            bs_theta,
            theta_var=math.radians(HEADING_FORCE_RESYNC_VAR_DEG) ** 2,
        )
        self._heading_resync_count += 1
        return True

    # ── Candidate smoother + envelope (§5.7-§5.8 / sim 1913-1967) ──

    def _compute_raw_candidate(self) -> Optional[Tuple[float, float]]:
        """Raw candidate goal in **odom frame**, computed from the live
        EKF ``θ_offset`` and current odom snapshot. As ``θ_offset``
        shifts (e.g. on heading-resync) or ``ekf.pos`` / ``last_odom_xy``
        update, the candidate moves in odom space; the EWMA + 5 m snap
        + 1/r envelope filter applied downstream is what gives
        self-correction.

        On the real robot we have no ground-truth heading, so the
        simulator's ``R(true_θ − θ_offset_est) · (goal_world − ekf.pos)``
        collapses to zero. The meaningful refining signal here is the
        GPS goal projected into odom frame: as ``θ_offset`` changes,
        the same world-frame goal lands at a different odom-frame point.
        This matches the publisher_tick's delta_world → delta_odom →
        goal_odom math (mod the smoother layer), so the smoothed
        candidate is what the publisher samples.

        Lock held. Manifest §5.6, §10.6.1.
        """
        active = self._active
        if active is None or active.goal_world_xy is None:
            return None
        # The bootstrap-done gate used to live here. It caused a
        # deadlock in the field: NAV2 won't translate the robot
        # without a goal, the handler's closed-form bootstrap can't
        # fit without translational GPS-vs-odom displacement, so the
        # robot sat idle waiting for a bootstrap that could never
        # complete. The GPS waypoint simulation in
        # Claude-Sandbox/GPS-Waypoint-Simulation does NOT have this
        # gate — its agents emit a candidate from the very first
        # tick using whatever the EKF's θ currently is (initially
        # high-variance), drive toward it (wrong direction at
        # first), and the resulting motion gives the EKF the
        # displacement data it needs to refine θ. The candidate
        # then converges to the true GPS goal as the agent moves.
        # Mirror that behaviour: publish whatever candidate we can
        # compute right now, even before bootstrap_done.
        if self._last_odom_xy is None:
            return None
        gx_w, gy_w = active.goal_world_xy
        ex, ey = self._ekf.pos_xy
        theta = self._ekf.theta
        c, s = math.cos(-theta), math.sin(-theta)
        dx_w, dy_w = gx_w - ex, gy_w - ey
        dx_o = c * dx_w - s * dy_w
        dy_o = s * dx_w + c * dy_w
        ox, oy = self._last_odom_xy
        return (ox + dx_o, oy + dy_o)

    def _update_candidate_smoother(self) -> None:
        """Lock held. EWMA + 5 m snap with a 1/r-envelope gate.

        The envelope is dormant when:
          * no goal active
          * smoother not yet initialized
          * envelope-suspension window from a moving-away trip is open

        Manifest §5.7-§5.8 / sim 1913-1967.
        """
        raw = self._compute_raw_candidate()
        if raw is None:
            return

        envelope_active = (
            self._smoothed_candidate is not None
            and self._now_s() >= self._envelope_suspended_until_s
        )
        if envelope_active and self._odom_distance_m > CANDIDATE_ENV_MIN_R_M:
            # Envelope: reject raw if it has swung far from the current
            # smoothed estimate (our best prior on the goal-in-odom).
            # Lever arm = robot-to-goal magnitude (frame-invariant under
            # the rigid map↔odom transform); compute it in world frame
            # against the active goal.
            r = self._odom_distance_m
            rx, ry = self._ekf.pos_xy
            active = self._active
            if active is not None and active.goal_world_xy is not None:
                gxw, gyw = active.goal_world_xy
                lever = math.hypot(rx - gxw, ry - gyw)
            else:
                lever = 0.0
            d_env = max(
                CANDIDATE_ENV_FLOOR_M,
                CANDIDATE_ENV_GAIN_M * lever / r,
            )
            sx, sy = self._smoothed_candidate
            d_raw = math.hypot(raw[0] - sx, raw[1] - sy)
            if d_raw > CANDIDATE_ENV_REJECT_K * d_env:
                self._cand_reject_count += 1
                return  # drop sample, hold previous smoothed

        if self._smoothed_candidate is None:
            self._smoothed_candidate = raw
            return
        sx, sy = self._smoothed_candidate
        dx = raw[0] - sx
        dy = raw[1] - sy
        if (dx * dx + dy * dy) > CANDIDATE_SNAP_M * CANDIDATE_SNAP_M:
            self._smoothed_candidate = raw
        else:
            a = CANDIDATE_SMOOTH_ALPHA
            self._smoothed_candidate = (sx + a * dx, sy + a * dy)

    # ── Moving-away detector (§5.9 / sim 2044-2077; NO stuck branch) ──

    def _update_moving_away(self) -> None:
        """Lock held. Pre-EKF trip wire: if the agent is *farther* from
        the goal at the end of a 3 s window than at the start by more
        than ``MOVING_AWAY_THRESHOLD_M``, suspend the 1/r envelope
        filter for ``MOVING_AWAY_ENV_SUSPEND_S`` and run a force-resync
        on the EKF heading. Estimator-side only — no controller side
        effect (Rule 7). Sim 2044-2077.

        Runs pre-bootstrap as well as post-bootstrap. Pre-bootstrap
        this is the safety net that catches a robot driving in the
        wrong direction because θ is still at its default (0) and the
        odom +Y axis isn't aligned with ENU north. The raw GPS
        ``d_goal`` calculation doesn't depend on EKF state, and
        ``_force_heading_resync`` calls ``closed_form_theta_window``
        which only needs 3 m of GPS baseline — well below the 5 m
        bootstrap threshold — so it can correct θ partway through
        the bootstrap drive instead of letting the robot run all the
        way to a phantom goal. Envelope suspension here is a no-op
        pre-bootstrap (the smoother isn't initialized yet); only the
        force-resync side effect matters.

        We do NOT port the STUCK branch (lines 2020-2031 +
        forward-thrust override 2262-2263); NAV2's bt_nav.xml has
        BackUp / GradientEscape / ClearCostmaps / Wait recoveries.
        """
        if self._active is None:
            return
        if self._last_gps_xy is None:
            return
        gx, gy = self._active.goal_world_xy
        gxr, gyr = self._last_gps_xy
        d_goal = math.hypot(gxr - gx, gyr - gy)
        now_s = self._now_s()
        self._dist_history.append((now_s, d_goal))
        cutoff = now_s - MOVING_AWAY_WINDOW_S
        while self._dist_history and self._dist_history[0][0] < cutoff:
            self._dist_history.popleft()
        if len(self._dist_history) < MOVING_AWAY_MIN_HISTORY_TICKS:
            return
        t_new, d_new = self._dist_history[-1]

        if now_s < self._envelope_suspended_until_s:
            return
        # The deque was just trimmed above so every remaining entry has
        # ``t >= now_s - MOVING_AWAY_WINDOW_S``. The oldest within-window
        # sample is therefore ``self._dist_history[0]`` — O(1) instead of
        # the previous linear scan. (The linear scan also used
        # ``target_t = t_new - WINDOW_S`` which is identical to the trim
        # cutoff since ``t_new`` was appended with ``now_s`` above.)
        oldest_t, oldest_d = self._dist_history[0]
        if (t_new - oldest_t) < MOVING_AWAY_WINDOW_COVERAGE * MOVING_AWAY_WINDOW_S:
            return
        delta = d_new - oldest_d   # +ve = farther from goal
        if delta > MOVING_AWAY_THRESHOLD_M:
            self._envelope_suspended_until_s = (
                now_s + MOVING_AWAY_ENV_SUSPEND_S
            )
            self._moving_away_event_count += 1
            # Per §13 #15: do NOT reset the smoother.
            resync_ok = self._force_heading_resync()
            self.get_logger().warn(
                f"moving-away detector fired: delta={delta:+.2f}m over "
                f"{t_new - oldest_t:.2f}s, count={self._moving_away_event_count}, "
                f"force_resync={'OK' if resync_ok else 'BLOCKED (insufficient baseline)'}"
            )

    def _update_local_world_divergence(self) -> None:
        """Lock held. Cross-check EKF-believed distance-to-goal against
        raw-GPS distance-to-goal. Both should decrease together as the
        robot makes real progress; if EKF distance shrinks far ahead of
        GPS distance, θ is wrong and the candidate is converging on a
        phantom goal (the failure mode the radial moving-away test
        misses for tangential drift).

        Forces a heading resync when the divergence crosses the
        threshold. Cooldowns and resets the per-goal baseline so we
        don't fire repeatedly during a single divergence event.
        Independent of bootstrap state.
        """
        active = self._active
        if active is None:
            return
        if self._last_gps_xy is None:
            return
        if self._now_s() < self._divergence_cooldown_until_s:
            return
        # Only meaningful for GPS goals — LOCAL/map goals have no
        # ``goal_world_xy`` reference frame in raw GPS coordinates.
        if active.goal_type != NavigateToWaypoint.Goal.GOAL_TYPE_GPS:
            return

        gx_w, gy_w = active.goal_world_xy
        ex, ey = self._ekf.pos_xy
        local_d = math.hypot(gx_w - ex, gy_w - ey)

        gxr, gyr = self._last_gps_xy
        world_d = math.hypot(gxr - gx_w, gyr - gy_w)

        # Lazy-init baselines on first valid sample.
        if active.local_d_start is None or active.world_d_start is None:
            active.local_d_start = local_d
            active.world_d_start = world_d
            return

        local_progress = active.local_d_start - local_d
        world_progress = active.world_d_start - world_d

        # Need substantial EKF-believed progress before evaluating.
        # Prevents tripping on noise during the first few meters.
        if local_progress < LOCAL_VS_WORLD_MIN_LOCAL_PROGRESS_M:
            return

        divergence = local_progress - world_progress
        if divergence > LOCAL_VS_WORLD_DIVERGENCE_M:
            # Same envelope-suspension side effect as moving-away —
            # lifts the 1/r filter so a corrected candidate can flow.
            self._envelope_suspended_until_s = (
                self._now_s() + MOVING_AWAY_ENV_SUSPEND_S
            )
            resync_ok = self._force_heading_resync()
            self.get_logger().warn(
                f"local↔world distance divergence: "
                f"local_progress={local_progress:.1f}m "
                f"world_progress={world_progress:.1f}m "
                f"divergence={divergence:.1f}m, "
                f"force_resync={'OK' if resync_ok else 'BLOCKED (insufficient baseline)'}"
            )
            self._divergence_cooldown_until_s = (
                self._now_s() + LOCAL_VS_WORLD_COOLDOWN_S
            )
            # Reset baselines so the next evaluation measures from
            # post-resync state, not the now-stale pre-correction one.
            active.local_d_start = local_d
            active.world_d_start = world_d

    # ── /goal_pose republish — gated on active goal (§5.12) ────────

    def _publish_local_map_goal(
        self,
        active: "_ActiveGoal",
        goal_map_xy: Tuple[float, float],
    ) -> None:
        """Publish a LOCAL/map-frame goal directly to /goal_pose.

        For LOCAL goals the goal's (x, y) is *already* in map frame —
        no projection, no TF lookup, no candidate smoother. This is the
        L1 (Wave 3) fast path that keeps GPS-pipeline machinery from
        corrupting an already-correct map-frame target.
        """
        gx_map, gy_map = float(goal_map_xy[0]), float(goal_map_xy[1])

        # Yaw at goal — use ``final_yaw`` if specified, else identity.
        # We deliberately do NOT auto-yaw toward the goal here: the
        # robot's map-frame pose is unknown at the publisher (we'd
        # need a separate map→base_link TF lookup), and identity is
        # NAV2-safe for a static map-frame target.
        if active.final_yaw is not None:
            yaw_goal = active.final_yaw
        else:
            yaw_goal = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw_goal)

        msg = PoseStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = gx_map
        msg.pose.position.y = gy_map
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._goal_pub.publish(msg)

        with self._lock:
            if self._active is not None and self._active.handle is active.handle:
                # For LOCAL goals goal_world_xy already IS the map-frame
                # goal; record the same pair to both world+map slots so
                # ``current_goal_in_map`` feedback works.
                self._active.last_published_goal_world = (gx_map, gy_map)
                self._active.last_published_goal_map = (gx_map, gy_map)

    def _publisher_tick(self) -> None:
        if self._active is None or self._publisher_disabled:
            return

        with self._lock:
            if self._active is None or self._publisher_disabled:
                return  # raced with cancel/preempt — anti-pattern §13 #13
            active = self._active
            ekf_x, ekf_y = self._ekf.pos_xy
            theta = self._ekf.theta
            theta_std_rad = self._ekf.theta_std_rad
            goal_world_xy = active.goal_world_xy
            # The smoother holds the candidate goal **in odom frame**
            # (see ``_compute_raw_candidate``). Use it as source-of-truth
            # so the EWMA + 5 m snap + 1/r envelope filter actually act
            # on what NAV2 sees. Fall back to a freshly-computed goal_odom
            # if the smoother isn't initialized yet.
            smoothed_odom = self._smoothed_candidate
            # Capture the robot's odom-frame position under the same lock
            # acquisition so the publisher snapshot is consistent. The
            # geometry below mixes odom-frame and world-frame quantities,
            # so all of them must come from one atomic read.
            odom_x, odom_y = self._last_odom_xy or (0.0, 0.0)

        # ── LOCAL (map-frame) goal fast path ─────────────────────────
        # For LOCAL goals the (x,y) is *already* in map frame; skip the
        # GPS-only world→odom→map projection + smoother pipeline and
        # publish directly. EKF heading is irrelevant here. TF is not
        # required (we already are in map). Manifest §4.3 routing rule.
        if active.goal_type == NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL:
            self._publish_local_map_goal(active, goal_world_xy)
            return
        # ── GPS goal: existing world→odom→map pipeline ───────────────

        # Stop-refining gate (§5.11) — hold the last published goal.
        ex, ey = ekf_x, ekf_y
        gx, gy = goal_world_xy
        d_goal = math.hypot(ex - gx, ey - gy)
        refinement_locked = d_goal < (STOP_REFINE_K * STOP_REFINE_SIGMA_GPS_M)
        if refinement_locked and active.last_published_goal_world is not None:
            return

        # θ-shift trigger (manifest §5.6 / §3.3)
        if active.last_published_theta is not None:
            theta_shift = abs(wrap_pi(theta - active.last_published_theta))
        else:
            theta_shift = math.inf

        # ── Goal projection (manifest §2.7, §5.6) ─────────────────────
        #
        # When the smoother has a value, it already lives in odom frame
        # (see ``_compute_raw_candidate``) — use it directly so the
        # EWMA / snap / envelope work has effect on the published goal.
        # Otherwise fall back to the live world→odom projection:
        #
        #   1) delta_world = goal_world − ekf_pos_world
        #      (goal's offset from the robot, in world frame)
        #   2) delta_odom  = R(−θ_offset) · delta_world
        #      (same offset, rotated into the odom frame)
        #   3) goal_odom   = last_odom_xy + delta_odom
        #      (robot's odom-frame position + offset → goal in odom)
        #   4) goal_map    = T_map_odom · goal_odom
        #      (transform odom → map via the TF lookup below)
        #
        # NB: ekf_pos_world (world frame) and last_odom_xy (odom frame)
        # are NOT added directly — only their corresponding offsets in
        # a common frame are combined. Combining the two robot positions
        # via a single rotation only collapses to identity at the origin
        # where both frames coincide; once the robot has driven any
        # distance, it produces a constant world-vs-odom anchor offset.
        if smoothed_odom is not None:
            goal_in_odom_x, goal_in_odom_y = smoothed_odom
        else:
            delta_world_x = goal_world_xy[0] - ekf_x
            delta_world_y = goal_world_xy[1] - ekf_y
            c = math.cos(-theta)
            s = math.sin(-theta)
            delta_odom_x = c * delta_world_x - s * delta_world_y
            delta_odom_y = s * delta_world_x + c * delta_world_y
            goal_in_odom_x = odom_x + delta_odom_x
            goal_in_odom_y = odom_y + delta_odom_y

        # Look up map → odom and project into map frame.
        try:
            tf_map_odom = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._odom_frame,
                Time(),
                rclpy.duration.Duration(seconds=self._tf_timeout_s),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().debug(
                f"map→odom TF unavailable; skipping /goal_pose publish"
            )
            return

        tx = float(tf_map_odom.transform.translation.x)
        ty = float(tf_map_odom.transform.translation.y)
        rq = tf_map_odom.transform.rotation
        yaw_mo = quat_to_yaw(rq.x, rq.y, rq.z, rq.w)
        c_mo = math.cos(yaw_mo)
        s_mo = math.sin(yaw_mo)
        gx_map = tx + c_mo * goal_in_odom_x - s_mo * goal_in_odom_y
        gy_map = ty + s_mo * goal_in_odom_x + c_mo * goal_in_odom_y

        # Yaw at goal — auto: face from current EKF position toward goal.
        if active.final_yaw is not None:
            yaw_goal = active.final_yaw
        else:
            ex_map = tx + c_mo * odom_x - s_mo * odom_y
            ey_map = ty + s_mo * odom_x + c_mo * odom_y
            vx = gx_map - ex_map
            vy = gy_map - ey_map
            yaw_goal = (
                math.atan2(vy, vx) if (vx * vx + vy * vy) > 1e-6 else 0.0
            )
        qx, qy, qz, qw = yaw_to_quat(yaw_goal)

        # Hard publish rate limit — but ONLY after bootstrap_done.
        # During bootstrap the handler cannot refine theta without
        # translational GPS-vs-odom displacement. If the rate limit
        # is in effect while bootstrap is incomplete the system can
        # deadlock: controller demands heading alignment, won't drive
        # forward without it, robot rotates in place, no translation,
        # bootstrap can't progress, goal stays in wrong direction,
        # controller keeps rotating. Observed in the field on
        # 2026-05-17 — first leg of a 3-waypoint mission, robot
        # spun for 90+ s with no progress.
        #
        # Letting the publisher run every tick (1 Hz) pre-bootstrap
        # gives NAV2 fresh goal_pose updates as the (still-converging)
        # heading_offset settles. The candidate jitter is enough for
        # DWB's trajectory rollout to find a candidate with non-zero
        # forward velocity, which translates the robot, which gives
        # the handler the displacement it needs to bootstrap.
        # Post-bootstrap, the rate limit kicks in and replan churn
        # is suppressed as designed.
        now_pub_s = self._now_s()
        last_map = active.last_published_goal_map
        last_t = active.last_published_t_s
        if last_map is not None and last_t is not None:
            since_last = now_pub_s - last_t
            if since_last < GOAL_REPUBLISH_HEARTBEAT_S:
                # Skip the publish — but still update the marker below
                # so RViz keeps showing the (held) candidate.
                with self._lock:
                    if self._active is not None:
                        self._active.last_published_theta = theta
                # Refresh the marker without re-publishing the goal.
                marker_only = Marker()
                marker_only.header.frame_id = self._map_frame
                marker_only.header.stamp = self.get_clock().now().to_msg()
                marker_only.ns = "gps_waypoint_candidate"
                marker_only.id = 0
                marker_only.type = Marker.SPHERE
                marker_only.action = Marker.ADD
                marker_only.pose.position.x = gx_map
                marker_only.pose.position.y = gy_map
                marker_only.pose.orientation.w = 1.0
                marker_only.scale.x = 0.4
                marker_only.scale.y = 0.4
                marker_only.scale.z = 0.4
                marker_only.color.r = 1.0
                marker_only.color.g = 0.6
                marker_only.color.b = 0.0
                marker_only.color.a = 0.8
                marker_only.lifetime = Duration(sec=2, nanosec=0)
                marker_only.frame_locked = True
                self._marker_pub.publish(marker_only)
                return

        msg = PoseStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = gx_map
        msg.pose.position.y = gy_map
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # Route: first publish of a leg AND periodic safety heartbeat
        # go to /goal_pose (kicks NavigateToPose action / refreshes the
        # action handle). All other in-mission corrections go to
        # /goal_update so the BT's GoalUpdater absorbs them without
        # canceling FollowPath — that's the fix for the stop/go.
        first_pub = active.last_published_goal_map is None
        heartbeat_due = (
            now_pub_s - active.last_goal_pose_t_s
            > GOAL_POSE_HEARTBEAT_S
        )
        if first_pub or heartbeat_due:
            self._goal_pub.publish(msg)
            active.last_goal_pose_t_s = now_pub_s
        else:
            self._goal_update_pub.publish(msg)

        # Marker for RViz — sphere at the smoothed candidate. The
        # 2 s lifetime auto-expires the marker if the publisher stops
        # ticking (goal terminated, TF lost, etc.) so RViz doesn't
        # leak a stale candidate forever; ``frame_locked=True`` keeps
        # it pinned to ``map`` between refreshes.
        marker = Marker()
        marker.header = msg.header
        marker.ns = "gps_waypoint_candidate"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = gx_map
        marker.pose.position.y = gy_map
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime = Duration(sec=2, nanosec=0)
        marker.frame_locked = True
        self._marker_pub.publish(marker)

        with self._lock:
            if self._active is not None:
                # ``last_published_goal_world`` is now used only as a
                # "have we published at all?" sentinel for the
                # stop-refining gate; store the canonical world goal.
                self._active.last_published_goal_world = goal_world_xy
                self._active.last_published_goal_map = (gx_map, gy_map)
                self._active.last_published_theta = theta
                self._active.last_published_t_s = now_pub_s

    # ── Always-on diagnostics (§4.2) ───────────────────────────────

    def _diag_tick(self) -> None:
        with self._lock:
            theta = self._ekf.theta
            theta_std_deg = math.degrees(self._ekf.theta_std_rad)
            payload = {
                "bootstrap_done": self._bootstrap_done,
                "ekf_updates": self._ekf.update_count,
                "ekf_rejects": self._ekf.rejected_count,
                "consecutive_rejects": self._ekf.consecutive_rejects,
                "heading_resync_count": self._heading_resync_count,
                "moving_away_events": self._moving_away_event_count,
                "candidate_rejects": self._cand_reject_count,
                "odom_distance_m": self._odom_distance_m,
                "gps_history_len": len(self._gps_history),
                "datum_lat": self._datum_lat,
                "datum_lon": self._datum_lon,
                "envelope_suspended": (
                    self._now_s() < self._envelope_suspended_until_s
                ),
                "active_goal": self._active is not None,
            }

        h = Float64()
        h.data = float(theta)
        self._heading_pub.publish(h)
        hs = Float64()
        hs.data = float(theta_std_deg)
        self._heading_std_pub.publish(hs)
        d = String()
        d.data = json.dumps(payload, default=float)
        self._debug_pub.publish(d)

    def _health_tick(self) -> None:
        """Emit /gps_waypoint/health = OK | DEGRADED <reason> | FAIL <reason>.

        Designed to fail loudly on the regression classes already
        observed in the field:

        - **FAIL ODOM_STALE**: /local_ekf/odom hasn't ticked in over
          HEALTH_ODOM_STALE_S. This is the "frozen pose" mode where
          ekf_global was 13 Hz of identical position values during
          a moving mission. Without fresh pose, goals get placed in
          phantom map XY.
        - **FAIL BOOTSTRAP_STUCK**: the robot has moved more than
          HEALTH_FAIL_BOOTSTRAP_AFTER_MOTION_M of cumulative odom
          distance but the handler hasn't bootstrapped — usually
          means /gps_fix isn't reaching this node or GPS gating is
          rejecting every sample.
        - **DEGRADED THETA_DRIFT_<deg>°/<window>s**: heading_offset
          has moved more than HEALTH_DEGRADED_THETA_DEG over the
          last HEALTH_WINDOW_S. Indicates ekf_local yaw is drifting,
          which makes goals migrate in map frame — the periodic
          left-turn artifact from the May 2026 outdoor run.
        - **FAIL THETA_DRIFT_…**: same metric exceeds the FAIL bound.

        Falls through to OK when none of the gates trip.
        """
        with self._lock:
            theta = self._ekf.theta
            bootstrap_done = self._bootstrap_done
            odom_stamp = self._last_odom_stamp_s
            odom_distance = self._odom_distance_m
            now_s = self._now_s()

        # Update rolling theta history. Append-only here; pruning is
        # cheap relative to the 1 Hz tick rate.
        self._theta_history.append((now_s, theta))
        cutoff = now_s - HEALTH_WINDOW_S
        while self._theta_history and self._theta_history[0][0] < cutoff:
            self._theta_history.popleft()

        # Build status. First failing gate wins; OK is the fallthrough.
        status: str = "OK"
        reason: str = ""

        odom_age = (now_s - odom_stamp) if odom_stamp is not None else None
        if odom_age is None or odom_age > HEALTH_ODOM_STALE_S:
            status = "FAIL"
            reason = (
                f"ODOM_STALE age="
                f"{'never' if odom_age is None else f'{odom_age:.1f}s'}"
            )
        elif (not bootstrap_done) and odom_distance > HEALTH_FAIL_BOOTSTRAP_AFTER_MOTION_M:
            status = "FAIL"
            reason = f"BOOTSTRAP_STUCK odom_dist={odom_distance:.1f}m"
        elif len(self._theta_history) >= 2:
            t0, theta0 = self._theta_history[0]
            t1, theta1 = self._theta_history[-1]
            window = max(t1 - t0, 1e-3)
            d_deg = abs(math.degrees(wrap_pi(theta1 - theta0)))
            if d_deg > HEALTH_FAIL_THETA_DEG:
                status = "FAIL"
                reason = f"THETA_DRIFT {d_deg:.1f}deg/{window:.0f}s"
            elif d_deg > HEALTH_DEGRADED_THETA_DEG:
                status = "DEGRADED"
                reason = f"THETA_DRIFT {d_deg:.1f}deg/{window:.0f}s"

        msg = String()
        msg.data = f"{status}{(' ' + reason) if reason else ''}"
        self._health_pub.publish(msg)

    # ── Action server ──────────────────────────────────────────────

    def _goal_callback(self, goal_request) -> GoalResponse:
        # Validate type ↔ frame_id pairing before accepting.
        gt = int(goal_request.goal_type)
        fid = goal_request.target.header.frame_id or ""
        if gt == NavigateToWaypoint.Goal.GOAL_TYPE_GPS and fid != "wgs84":
            self.get_logger().warn(
                f"rejecting GPS goal: frame_id='{fid}' (expected 'wgs84')"
            )
            return GoalResponse.REJECT
        if gt == NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL:
            # L1 scope decision (Wave 3): local goals are accepted ONLY
            # in map frame. Odom-frame local goals would require a
            # separate `T_map_odom`-projected publish path that doesn't
            # round-trip through the world→odom→map / smoother pipeline
            # built for GPS goals. Rather than ship a half-working
            # version, reject odom-frame local goals at acceptance with
            # STATUS_INVALID_GOAL. See goal_world_xy comment in
            # _ActiveGoal for the field-reuse rationale.
            if fid != self._map_frame:
                self.get_logger().warn(
                    f"rejecting LOCAL goal: frame_id='{fid}' "
                    f"(only '{self._map_frame}' is supported; "
                    f"odom-frame local goals are not yet implemented)"
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        """Anti-pattern §13 #13 — *stop the publisher first*, then accept.

        We can't call ``goal_handle.canceled()`` here (only the owning
        ``execute_callback`` may finalize the handle), but we *can*
        force-disable the 1 Hz ``/goal_pose`` republisher synchronously
        so NAV2 doesn't get one more stale goal between this ACK and
        the loop's next ``is_cancel_requested`` poll. The
        ``execute_callback`` loop will then enter ``_terminate`` on its
        next tick, which clears ``_active`` and returns the cancel
        result with the proper terminal_status.
        """
        with self._lock:
            self._publisher_disabled = True
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Coroutine-style goal execution. Runs in the action callback
        group (Reentrant) so cancel can interrupt mid-loop.

        Preempt-with-cancel (§5.14): if another goal is already active,
        *signal* it via ``prior.preempt_requested`` (we cannot call
        ``prior.handle.canceled()`` from here — only the prior's own
        execute_callback may transition its goal handle, and
        ``is_cancel_requested`` is only set by client cancellations,
        never by an internal preempt). The prior's loop polls the flag
        on its next tick, routes through ``_terminate`` with
        ``STATUS_PREEMPTED``, calls ``goal_handle.canceled()`` itself
        (the closest valid terminal in rclpy_action's state machine —
        there is no ``preempted()``), and signals ``preempt_done``. We
        wait on that event, then accept the new goal.
        """
        # Preempt prior goal if any.
        with self._lock:
            prior = self._active
            if prior is not None and prior.handle is not goal_handle:
                prior.preempt_requested = True
                # Stop the publisher immediately so NAV2 doesn't see
                # one more stale goal between this branch and the
                # prior loop's next tick (anti-pattern §13 #13).
                self._publisher_disabled = True
                prior_event = prior.preempt_done
            else:
                prior_event = None
        if prior_event is not None:
            self.get_logger().info("preempting prior goal")
            # Wait for the prior loop to finalize. Loop period is
            # ``1/feedback_hz`` (≈0.5 s at 2 Hz), so 1.0 s gives the
            # prior at least one full tick to notice + return.
            if not prior_event.wait(timeout=1.0):
                self.get_logger().warn(
                    "prior goal did not terminate within 1.0 s; "
                    "proceeding anyway"
                )
            # Defensive: clear the slot if the prior somehow didn't
            # call _terminate (shouldn't happen — preserve invariant).
            with self._lock:
                if self._active is prior:
                    self._active = None
                    self._smoothed_candidate = None
                    self._dist_history.clear()

        # Project the goal into world meters around our datum.
        goal_msg: NavigateToWaypoint.Goal = goal_handle.request
        gt = int(goal_msg.goal_type)
        fid = goal_msg.target.header.frame_id or ""
        radius = float(goal_msg.success_radius_m) or self._success_radius_default
        # Yaw extraction — identity quaternion ⇒ auto.
        q = goal_msg.target.pose.orientation
        ident = (
            abs(q.x) < 1e-6
            and abs(q.y) < 1e-6
            and abs(q.z) < 1e-6
            and abs(q.w - 1.0) < 1e-6
        )
        final_yaw: Optional[float] = None if ident else quat_to_yaw(
            q.x, q.y, q.z, q.w
        )

        goal_lat_lon: Optional[Tuple[float, float]] = None
        goal_world_xy: Tuple[float, float]
        local_input_frame: Optional[str] = None

        if gt == NavigateToWaypoint.Goal.GOAL_TYPE_GPS:
            if fid != "wgs84":
                return self._abort(
                    goal_handle,
                    NavigateToWaypoint.Result.STATUS_INVALID_GOAL,
                    f"GPS goal requires frame_id='wgs84', got '{fid}'",
                )
            lat = float(goal_msg.target.pose.position.y)
            lon = float(goal_msg.target.pose.position.x)
            goal_lat_lon = (lat, lon)
            with self._lock:
                if self._datum_lat is None:
                    return self._abort(
                        goal_handle,
                        NavigateToWaypoint.Result.STATUS_GPS_LOST,
                        "no GPS datum yet — wait for first /gps_fix",
                    )
                gx, gy = latlon_to_local(
                    lat, lon, self._datum_lat, self._datum_lon
                )
            goal_world_xy = (gx, gy)
        elif gt == NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL:
            # L1 scope: only map-frame local goals are supported. The
            # GPS publisher path (world→odom→map + smoother) is wrong
            # for "the goal is already in map frame", and odom-frame
            # local goals would need a separate T_map_odom publish path
            # — out of scope here.
            if fid == self._map_frame:
                local_input_frame = self._map_frame
            elif fid == self._odom_frame:
                # Defense-in-depth: _goal_callback already rejects this,
                # but if a caller bypasses it (e.g. internal call),
                # fail clean rather than silently treat odom coords as
                # map coords.
                return self._abort(
                    goal_handle,
                    NavigateToWaypoint.Result.STATUS_INVALID_GOAL,
                    "odom-frame local goals not yet supported; use map frame",
                )
            else:
                return self._abort(
                    goal_handle,
                    NavigateToWaypoint.Result.STATUS_INVALID_GOAL,
                    f"LOCAL goal requires frame_id='{self._map_frame}', got '{fid}'",
                )
            lx = float(goal_msg.target.pose.position.x)
            ly = float(goal_msg.target.pose.position.y)
            # Stored in goal_world_xy slot, but interpreted as map-frame
            # XY for LOCAL goals (publisher branches on goal_type).
            goal_world_xy = (lx, ly)
        else:
            return self._abort(
                goal_handle,
                NavigateToWaypoint.Result.STATUS_INVALID_GOAL,
                f"unknown goal_type {gt}",
            )

        active = _ActiveGoal(
            handle=goal_handle,
            goal_type=gt,
            frame_id=fid,
            success_radius_m=radius,
            final_yaw=final_yaw,
            goal_lat_lon=goal_lat_lon,
            goal_world_xy=goal_world_xy,
            local_input_frame=local_input_frame,
            started_ros_s=self._now_s(),
        )
        with self._lock:
            self._active = active
            self._smoothed_candidate = None
            self._dist_history.clear()
            self._envelope_suspended_until_s = 0.0
            # New goal — re-enable the 1 Hz /goal_pose republisher
            # (it was disabled by the prior goal's cancel/preempt).
            self._publisher_disabled = False

        # Feedback loop @ FEEDBACK_HZ.
        period = 1.0 / max(self._feedback_hz, 0.5)
        try:
            while rclpy.ok():
                # Internal preempt by a newer goal (§5.14). Routed
                # through ``canceled()`` since rclpy_action has no
                # ``preempted()``; STATUS_PREEMPTED travels in the
                # Result message instead (manifest §5.14, §13).
                if active.preempt_requested:
                    return self._terminate(
                        goal_handle,
                        NavigateToWaypoint.Result.STATUS_PREEMPTED,
                        "superseded by newer goal",
                    )
                # Client-initiated cancel via ``cancel_goal``.
                if goal_handle.is_cancel_requested:
                    return self._terminate(
                        goal_handle,
                        NavigateToWaypoint.Result.STATUS_CANCELED,
                        "client canceled",
                    )

                with self._lock:
                    ekf_xy = self._ekf.pos_xy
                    theta_std_deg = math.degrees(self._ekf.theta_std_rad)
                    theta_deg = math.degrees(self._ekf.theta)
                    bootstrap_done = self._bootstrap_done
                    last_gps_stamp = self._last_gps_stamp_s
                    last_pub_map = active.last_published_goal_map

                # Track distance traveled & peak θ-std.
                if active.last_pos_xy is not None:
                    active.distance_traveled_m += math.hypot(
                        ekf_xy[0] - active.last_pos_xy[0],
                        ekf_xy[1] - active.last_pos_xy[1],
                    )
                active.last_pos_xy = ekf_xy
                if theta_std_deg > active.peak_theta_std_deg:
                    active.peak_theta_std_deg = theta_std_deg

                # Distance to goal — frame must match goal's native frame.
                gx, gy = active.goal_world_xy
                if active.goal_type == NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL:
                    # LOCAL/map goal: goal_world_xy IS in map frame; need
                    # robot's map-frame position. Try map → base_link
                    # via TF; fall back to ekf_xy (world frame) on TF
                    # failure with imprecision flagged. LOCAL-goal use is
                    # an edge case for the GPS waypoint handler — see L1
                    # scope note in ``_goal_callback``.
                    robot_map_xy = self._lookup_robot_in_map()
                    if robot_map_xy is not None:
                        d_goal = math.hypot(
                            robot_map_xy[0] - gx, robot_map_xy[1] - gy
                        )
                    else:
                        # Imprecise fallback (world ↔ map offset bias).
                        d_goal = math.hypot(
                            ekf_xy[0] - gx, ekf_xy[1] - gy
                        )
                else:
                    # GPS goal: both quantities are in world frame.
                    d_goal = math.hypot(ekf_xy[0] - gx, ekf_xy[1] - gy)
                refinement_locked = d_goal < (
                    STOP_REFINE_K * STOP_REFINE_SIGMA_GPS_M
                )

                # Liveness telemetry. We deliberately do NOT abort on
                # GPS staleness — operator policy is "goals run until
                # finished." If /gps_fix goes silent the EKF coasts on
                # odometry; the action surfaces the staleness via
                # ``feedback.gps_connected`` so callers can see it
                # without terminating the goal.
                _gps_stale = (
                    active.goal_type == NavigateToWaypoint.Goal.GOAL_TYPE_GPS
                    and last_gps_stamp is not None
                    and (self._now_s() - last_gps_stamp)
                    > self._gps_stale_timeout_s
                )
                if _gps_stale:
                    # Throttle the warn so a long stale window doesn't
                    # spam — once per stale-timeout window is enough.
                    self.get_logger().warn(
                        f"/gps_fix stale > {self._gps_stale_timeout_s:.1f} s "
                        f"(continuing; goal will not abort)",
                        throttle_duration_sec=self._gps_stale_timeout_s,
                    )

                # Feedback.
                fb = NavigateToWaypoint.Feedback()
                fb.distance_to_goal_m = float(d_goal)
                fb.ekf_theta_deg = float(theta_deg)
                fb.ekf_theta_std_deg = float(theta_std_deg)
                fb.gps_connected = last_gps_stamp is not None and (
                    (self._now_s() - last_gps_stamp)
                    < self._gps_stale_timeout_s
                )
                fb.refinement_locked = bool(refinement_locked)
                cg = PoseStamped()
                cg.header.frame_id = self._map_frame
                cg.header.stamp = self.get_clock().now().to_msg()
                # ``current_goal_in_map`` carries the same map-frame
                # pose the publisher last sent on /goal_pose. Until the
                # publisher has produced one (TF unavailable, refinement
                # locked early), zeros — clients should treat that as
                # "no published goal yet" via the timestamp alignment.
                if last_pub_map is not None:
                    cg.pose.position.x = float(last_pub_map[0])
                    cg.pose.position.y = float(last_pub_map[1])
                cg.pose.orientation.w = 1.0
                fb.current_goal_in_map = cg
                goal_handle.publish_feedback(fb)

                # Arrival. For GPS goals, gate on bootstrap (the EKF
                # must have its θ_offset before we trust the world-frame
                # arrival check). LOCAL/map goals don't depend on the
                # EKF heading at all — base arrival on the map-frame
                # distance directly.
                arrival_ready = (
                    bootstrap_done
                    if active.goal_type
                    == NavigateToWaypoint.Goal.GOAL_TYPE_GPS
                    else True
                )
                if arrival_ready and d_goal < active.success_radius_m:
                    return self._terminate(
                        goal_handle,
                        NavigateToWaypoint.Result.STATUS_SUCCESS,
                        "",
                    )

                # NOTE: ``time.sleep`` blocks the executor thread that
                # is running this ``execute_callback``. That's
                # acceptable here because (a) we're on a
                # ``MultiThreadedExecutor(num_threads=4)`` so the EKF
                # callback / publisher / services keep ticking on the
                # other threads, and (b) the action server lives in a
                # ``ReentrantCallbackGroup`` so cancel/preempt
                # requests can preempt this thread mid-sleep on their
                # own callback. Switching to ``rclpy.Rate`` would also
                # work but ``Rate`` itself blocks; the only real win
                # would be a coroutine-friendly sleep, which would
                # require converting this to ``async def`` — out of
                # scope for this fix.
                time.sleep(period)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"execute_callback exception: {exc}")
            return self._terminate(
                goal_handle,
                NavigateToWaypoint.Result.STATUS_ABORTED,
                f"exception: {exc}",
            )

        return self._terminate(
            goal_handle,
            NavigateToWaypoint.Result.STATUS_ABORTED,
            "executor shutdown",
        )

    # ── Terminal helpers ────────────────────────────────────────────

    def _abort(
        self,
        goal_handle: ServerGoalHandle,
        status: int,
        reason: str,
    ) -> NavigateToWaypoint.Result:
        """Reject before the loop ever starts. No active slot to clear."""
        result = NavigateToWaypoint.Result()
        result.terminal_status = int(status)
        result.failure_reason = reason
        result.succeeded = False
        goal_handle.abort()
        return result

    def _terminate(
        self,
        goal_handle: ServerGoalHandle,
        status: int,
        reason: str,
    ) -> NavigateToWaypoint.Result:
        """Stop the publisher (clear active slot) BEFORE returning the
        terminal — anti-pattern §13 #13. Also signals
        ``active.preempt_done`` so a waiting newer goal proceeds
        promptly (§5.14)."""
        with self._lock:
            active = self._active
            ekf_xy = self._ekf.pos_xy
            theta_std_deg = math.degrees(self._ekf.theta_std_rad)
            theta = self._ekf.theta
            datum_lat = self._datum_lat
            datum_lon = self._datum_lon
            ekf_updates = self._ekf.update_count
            gps_outlier_rejects = self._ekf.rejected_count
            heading_resyncs = self._heading_resync_count
            # Only clear _active if it still belongs to *this* goal.
            # A newer goal's preempt branch may have already taken
            # over and reseated the slot — don't trample it.
            if active is not None and active.handle is goal_handle:
                self._active = None  # ← stops the publisher tick
                self._smoothed_candidate = None
                self._dist_history.clear()
                self._publisher_disabled = True

        result = NavigateToWaypoint.Result()
        result.terminal_status = int(status)
        result.succeeded = (
            status == NavigateToWaypoint.Result.STATUS_SUCCESS
        )
        if active is not None:
            gx, gy = active.goal_world_xy
            d_goal = math.hypot(ekf_xy[0] - gx, ekf_xy[1] - gy)
            result.final_distance_m = float(d_goal)
            result.distance_traveled_m = float(active.distance_traveled_m)
            result.peak_theta_offset_std_deg = float(
                max(active.peak_theta_std_deg, theta_std_deg)
            )
            result.elapsed_s = float(self._now_s() - active.started_ros_s)
            heading_err = math.degrees(
                math.atan2(gy - ekf_xy[1], gx - ekf_xy[0])
            ) - math.degrees(theta)
            result.final_heading_err_deg = float(wrap_pi(math.radians(heading_err)) * 180.0 / math.pi)
        if datum_lat is not None and datum_lon is not None:
            lat, lon = local_to_latlon(
                ekf_xy[0], ekf_xy[1], datum_lat, datum_lon
            )
            result.final_latitude = float(lat)
            result.final_longitude = float(lon)
        result.ekf_updates = int(ekf_updates)
        result.gps_outlier_rejects = int(gps_outlier_rejects)
        result.heading_resyncs_fired = int(heading_resyncs)
        result.failure_reason = reason

        # rclpy_action's ServerGoalHandle has only succeed() / abort()
        # / canceled() — no preempted(). STATUS_PREEMPTED is an
        # action-message enum, not a state-machine state, so we use
        # ``terminal_status`` in the Result to carry the preempt
        # semantics and pick a state-machine transition that's
        # actually legal from EXECUTING.
        #
        # Critical: ``goal_handle.canceled()`` is ONLY valid from the
        # CANCELING state, which the handle only enters when a CLIENT
        # invokes cancel_goal. For *internal* preemption (a newer goal
        # arrived) the handle is still in EXECUTING — calling
        # ``canceled()`` then throws ``invalid transition from state
        # EXECUTING with event CANCELED`` and bubbles up as the
        # ``execute_callback exception`` we saw repeating every ~5 s.
        # Use ``goal_handle.is_cancel_requested`` to disambiguate.
        if result.succeeded:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            # client cancel — handle is already in CANCELING
            goal_handle.canceled()
        else:
            # internal preempt or abort — handle is in EXECUTING
            goal_handle.abort()

        # Release any newer goal that's waiting in its preempt branch.
        if active is not None and active.preempt_done is not None:
            active.preempt_done.set()

        # Publish a DELETE marker so RViz removes the candidate-goal
        # sphere immediately on terminal — without this, the 2 s
        # lifetime would leave a ghost behind. ns/id must match the
        # ADD marker in ``_publisher_tick``.
        delete_marker = Marker()
        delete_marker.header.frame_id = self._map_frame
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "gps_waypoint_candidate"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETE
        self._marker_pub.publish(delete_marker)
        return result

    # ── Shutdown ───────────────────────────────────────────────────

    def _on_shutdown(self) -> None:
        """Cleanly abort any in-flight action goal so clients don't
        hang on their result future when the node goes down (Ctrl+C
        or ``rclpy.shutdown()``)."""
        with self._lock:
            active = self._active
        if active is None:
            return
        self.get_logger().info("Node shutting down — aborting active goal.")
        try:
            if active.handle.is_active:
                self._terminate(
                    active.handle,
                    NavigateToWaypoint.Result.STATUS_ABORTED,
                    "Node shutdown.",
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Shutdown abort raised: {exc}")

    # ── Services ───────────────────────────────────────────────────

    def _srv_gps_to_local(
        self,
        request: GpsToLocal.Request,
        response: GpsToLocal.Response,
    ) -> GpsToLocal.Response:
        with self._lock:
            response.estimate_valid = bool(self._bootstrap_done)
            response.theta_offset_std_deg = float(
                math.degrees(self._ekf.theta_std_rad)
            )
            if self._datum_lat is None:
                response.x = 0.0
                response.y = 0.0
                response.estimate_valid = False
                return response
            wx, wy = latlon_to_local(
                float(request.latitude),
                float(request.longitude),
                self._datum_lat,
                self._datum_lon,
            )
            # Apply current θ to express world XY in odom XY:
            # odom_xy = R(-θ) · (world_xy - ekf_pos_world) + odom_pos.
            ekf_x, ekf_y = self._ekf.pos_xy
            odom_x, odom_y = self._last_odom_xy or (0.0, 0.0)
            theta = self._ekf.theta
            c = math.cos(-theta)
            s = math.sin(-theta)
            dx = wx - ekf_x
            dy = wy - ekf_y
            response.x = float(odom_x + c * dx - s * dy)
            response.y = float(odom_y + s * dx + c * dy)
        return response

    def _srv_local_to_gps(
        self,
        request: LocalToGps.Request,
        response: LocalToGps.Response,
    ) -> LocalToGps.Response:
        with self._lock:
            response.estimate_valid = bool(self._bootstrap_done)
            response.theta_offset_std_deg = float(
                math.degrees(self._ekf.theta_std_rad)
            )
            if self._datum_lat is None:
                response.latitude = 0.0
                response.longitude = 0.0
                response.estimate_valid = False
                return response
            ekf_x, ekf_y = self._ekf.pos_xy
            odom_x, odom_y = self._last_odom_xy or (0.0, 0.0)
            theta = self._ekf.theta
            c = math.cos(theta)
            s = math.sin(theta)
            # world_xy = ekf_pos + R(θ) · (odom_xy - odom_pos).
            dx = float(request.x) - odom_x
            dy = float(request.y) - odom_y
            wx = ekf_x + c * dx - s * dy
            wy = ekf_y + s * dx + c * dy
            lat, lon = local_to_latlon(
                wx, wy, self._datum_lat, self._datum_lon
            )
            response.latitude = float(lat)
            response.longitude = float(lon)
        return response


# ── Entry point ─────────────────────────────────────────────────────


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GpsHandlerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Abort any in-flight goal so the action client's result
        # future resolves instead of hanging on shutdown.
        node._on_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
