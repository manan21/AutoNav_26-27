"""Pure-Python / pure-numpy EKF for the magnetometer-less GPS waypoint
handler.

Ported from the simulator at
``/Users/nathanfikes/Projects/Claude-Sandbox/GPS-Waypoint-Simulation/src/gps_sim_gui.py``
(``class GPSEKF`` lines 971-1086, ``_bootstrap_theta`` lines 1608-1639,
``_closed_form_theta_window`` lines 1641-1679). No ROS imports — this
module is unit-testable in isolation.

Why this filter exists:
    The robot has no magnetometer (slam/config/dual_ekf_navsat_params.yaml
    sets magnetic_declination_radians: 0.0 and use_odometry_yaw: true).
    The unknown rotation between the ``odom`` frame and the world / GPS
    frame is the EKF's third state: ``θ``. Position is predicted from
    odom deltas and corrected by GPS. ``θ`` is corrected by direct
    measurements from GPS course-over-ground minus IMU-fused odom yaw in
    ``gps_handler_node``; GPS position updates intentionally do not infer
    heading from encoder-biased odom deltas.

    The pure EKF cannot escape a 180°-wrong cold start, so a closed-form
    weighted circular mean of ``atan2(Δgps) − atan2(Δodom)`` runs while
    the robot has traveled less than 5 m. After that we hand off to the
    EKF.
"""

from __future__ import annotations

import itertools
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


# ── Constants (mirrored from plan_manifest §3 / survey §7) ──────────
EKF_GPS_SIGMA: float = 1.2
"""What the EKF expects GPS σ to be, m. Intentionally larger than the
raw GPS_NOISE_STD (0.30) so it absorbs slow bias drift."""

EKF_GATE_CHI2: float = 50.0
"""Mahalanobis² gate. Calibrated against bias drift + outliers; tighter
gates rejected too many normal samples."""

EKF_REJ_STREAK_RESET: int = 25
"""After 25 consecutive rejections (~2.5 s @ 10 Hz GPS) force-accept
the next reading and re-inflate position covariance."""

EKF_POS_VAR_FLOOR: float = 1.0
"""Floor on EKF position variance, m² (1.0 m one-sigma). Keeps the
filter from claiming sub-decimeter certainty."""

HEADING_FIT_MAGRATIO_MAX: float = 3.0
"""Max ratio between |Δgps| and |Δodom| accepted in the closed-form
heading fit. Filters out spoofer-pinned (ratio → 0) and jam-degraded
(ratio → ∞) sample pairs."""

# Process / measurement noise
_Q_POS: float = 1.0e-3
_Q_THETA: float = 1.0e-4


def wrap_pi(angle: float) -> float:
    """Wrap an angle to ``(-π, π]``."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class GpsEkf:
    """3-state EKF on ``[x_world, y_world, θ]``.

    ``θ`` is the rotation odom → world. It substitutes for the
    magnetometer reading the robot does not have.
    """

    def __init__(
        self,
        x0: float = 0.0,
        y0: float = 0.0,
        q_pos: float = _Q_POS,
        q_theta: float = _Q_THETA,
        r_gps: float = EKF_GPS_SIGMA,
        theta0: float = 0.0,
        theta_var0: Optional[float] = None,
    ) -> None:
        self.x: np.ndarray = np.array([x0, y0, theta0], dtype=float)
        if theta_var0 is None:
            theta_var0 = math.pi ** 2  # full ±π uncertainty at cold start
        self.P: np.ndarray = np.diag(
            [r_gps ** 2, r_gps ** 2, theta_var0]
        ).astype(float)
        self._Q: np.ndarray = np.diag(
            [q_pos ** 2, q_pos ** 2, q_theta ** 2]
        )
        self._R: np.ndarray = np.diag([r_gps ** 2, r_gps ** 2])
        self._r_gps: float = float(r_gps)

        # Diagnostics
        self.last_innovation: Tuple[float, float] = (0.0, 0.0)
        self.last_mahalanobis: float = 0.0
        self.rejected_count: int = 0
        self.update_count: int = 0
        self.consecutive_rejects: int = 0

    # ── Predict ────────────────────────────────────────────────────
    def predict(self, dxo: float, dyo: float, dt: float) -> None:
        """Roll the world-position forward by an odom-frame delta.

        Math (manifest §2.4 / sim lines 1015-1034)::

            x_w' = x_w + cos(θ) Δx_o − sin(θ) Δy_o
            y_w' = y_w + sin(θ) Δx_o + cos(θ) Δy_o
            θ'   = θ
        """
        c = math.cos(self.x[2])
        s = math.sin(self.x[2])
        self.x[0] += c * dxo - s * dyo
        self.x[1] += s * dxo + c * dyo
        # F = ∂f/∂x linearization around the current θ.
        #
        # The off-diagonal F[0,2] / F[1,2] entries are the mechanism
        # by which a GPS-position update could rotate θ via
        # correlation: GPS reduces position uncertainty, and the
        # correlation built up during predict propagates that
        # reduction into θ. That sounds useful, but the ``dxo, dyo``
        # we feed here are encoder-derived odom deltas — when the
        # encoder is yaw-biased, the EKF "explains" the GPS-vs-predict
        # mismatch by rotating θ in the direction of the bias. The
        # result is that θ tracks the encoder drift rather than world
        # truth.
        #
        # Zeroing F[0,2] and F[1,2] decouples θ from position
        # entirely: GPS updates correct (x_w, y_w) only, and θ is
        # updated exclusively by direct ``update_theta_measurement``
        # calls sourced from GPS course-over-ground + IMU yaw (see
        # ``gps_handler_node._inject_gps_cog_theta_measurement``).
        # No ODOM angle anywhere in the θ chain.
        # F is intentionally identity here, so F @ P @ F.T is just P.
        # Avoid allocating/multiplying three small numpy matrices on every
        # odom tick.
        self.P += self._Q * float(dt)
        # Position-variance floor — see EKF_POS_VAR_FLOOR docstring.
        # When clamping a diagonal entry up, scale the corresponding row
        # and column off-diagonals by sqrt(new/old) so correlation
        # coefficients ρ_ij = P[i,j] / sqrt(P[i,i]·P[j,j]) are preserved
        # and the matrix remains PSD (Cauchy-Schwarz holds).
        for i in (0, 1):
            if self.P[i, i] < EKF_POS_VAR_FLOOR:
                old_var = float(self.P[i, i])
                new_var = EKF_POS_VAR_FLOOR
                scale = math.sqrt(new_var / max(old_var, 1e-12))
                for j in range(self.P.shape[0]):
                    if j != i:
                        self.P[i, j] *= scale
                        self.P[j, i] *= scale
                self.P[i, i] = new_var

    # ── Update ─────────────────────────────────────────────────────
    def update(
        self,
        zx: float,
        zy: float,
        gate_chi2: float = EKF_GATE_CHI2,
    ) -> bool:
        """Standard 2-D position Kalman update with Mahalanobis gating.

        Returns ``True`` iff the sample was accepted. Implements the
        sim lines 1046-1070, plus the consecutive-reject-streak counter
        used by lock-in recovery (see ``force_accept_next``).
        """
        y0 = float(zx) - float(self.x[0])
        y1 = float(zy) - float(self.x[1])
        s00 = float(self.P[0, 0] + self._R[0, 0])
        s01 = float(self.P[0, 1])
        s10 = float(self.P[1, 0])
        s11 = float(self.P[1, 1] + self._R[1, 1])
        det = s00 * s11 - s01 * s10
        if not math.isfinite(det) or abs(det) < 1e-18:
            return False

        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det
        sy0 = inv00 * y0 + inv01 * y1
        sy1 = inv10 * y0 + inv11 * y1
        m2 = y0 * sy0 + y1 * sy1
        self.last_innovation = (y0, y1)
        self.last_mahalanobis = m2
        if m2 > gate_chi2:
            self.rejected_count += 1
            self.consecutive_rejects += 1
            return False

        p00 = float(self.P[0, 0])
        p01 = float(self.P[0, 1])
        p02 = float(self.P[0, 2])
        p10 = float(self.P[1, 0])
        p11 = float(self.P[1, 1])
        p12 = float(self.P[1, 2])
        p20 = float(self.P[2, 0])
        p21 = float(self.P[2, 1])
        p22 = float(self.P[2, 2])

        k00 = p00 * inv00 + p01 * inv10
        k01 = p00 * inv01 + p01 * inv11
        k10 = p10 * inv00 + p11 * inv10
        k11 = p10 * inv01 + p11 * inv11
        k20 = p20 * inv00 + p21 * inv10
        k21 = p20 * inv01 + p21 * inv11

        self.x[0] += k00 * y0 + k01 * y1
        self.x[1] += k10 * y0 + k11 * y1
        self.x[2] = wrap_pi(self.x[2] + k20 * y0 + k21 * y1)

        # H selects rows 0 and 1, so (I - K H) P == P - K @ P[:2, :].
        self.P[0, 0] -= k00 * p00 + k01 * p10
        self.P[0, 1] -= k00 * p01 + k01 * p11
        self.P[0, 2] -= k00 * p02 + k01 * p12
        self.P[1, 0] -= k10 * p00 + k11 * p10
        self.P[1, 1] -= k10 * p01 + k11 * p11
        self.P[1, 2] -= k10 * p02 + k11 * p12
        self.P[2, 0] -= k20 * p00 + k21 * p10
        self.P[2, 1] -= k20 * p01 + k21 * p11
        self.P[2, 2] -= k20 * p02 + k21 * p12

        p01_new = 0.5 * (self.P[0, 1] + self.P[1, 0])
        p02_new = 0.5 * (self.P[0, 2] + self.P[2, 0])
        p12_new = 0.5 * (self.P[1, 2] + self.P[2, 1])
        self.P[0, 1] = self.P[1, 0] = p01_new
        self.P[0, 2] = self.P[2, 0] = p02_new
        self.P[1, 2] = self.P[2, 1] = p12_new
        # Floor the position variance HERE, immediately after the Kalman
        # gain shrinks it. The predict step also enforces this floor, but
        # update() can drive P[0,0]/P[1,1] far below it in a single step
        # — and if a biased GPS sample is consistently accepted, the
        # gain on subsequent samples collapses and the EKF locks to the
        # bias before predict() has a chance to reinflate. Mirror the
        # off-diagonal scaling logic from predict() to preserve correlation.
        for i in (0, 1):
            if self.P[i, i] < EKF_POS_VAR_FLOOR:
                old_var = float(self.P[i, i])
                if old_var > 1e-12:
                    scale = math.sqrt(EKF_POS_VAR_FLOOR / old_var)
                    for j in range(self.P.shape[0]):
                        if j != i:
                            self.P[i, j] *= scale
                            self.P[j, i] *= scale
                self.P[i, i] = EKF_POS_VAR_FLOOR
        # Keep θ in (-π, π].
        self.x[2] = wrap_pi(self.x[2])
        self.update_count += 1
        self.consecutive_rejects = 0
        return True

    def force_accept_next(self) -> None:
        """Lock-in recovery: re-inflate position covariance to
        ``EKF_GPS_SIGMA²`` so the next GPS sample drags the estimate
        back toward truth even if the gate would normally reject it.

        Called by the node after ``EKF_REJ_STREAK_RESET`` consecutive
        rejections. Manifest §3.4 / sim §10.2 lock-in recovery.
        """
        self.P[0, 0] = max(self.P[0, 0], self._r_gps ** 2)
        self.P[1, 1] = max(self.P[1, 1], self._r_gps ** 2)
        self.consecutive_rejects = 0

    # ── θ reset ────────────────────────────────────────────────────
    def reset_theta(
        self, theta: float, theta_var: Optional[float] = None
    ) -> None:
        """Snap-replace ``θ`` and decorrelate it from x, y.

        Used after the closed-form bootstrap and during heading-resync.
        Sim lines 1036-1044.
        """
        self.x[2] = wrap_pi(float(theta))
        if theta_var is None:
            theta_var = math.radians(20.0) ** 2
        self.P[2, :] = 0.0
        self.P[:, 2] = 0.0
        self.P[2, 2] = float(theta_var)

    def update_theta_measurement(
        self, theta_obs: float, theta_meas_std: float
    ) -> bool:
        """Scalar Kalman update on ``θ`` as a direct measurement.

        Use this AFTER bootstrap completes, where we want successive
        observations weighed against the EKF's accumulated confidence
        rather than snap-replacing it. As ``P[2,2]`` shrinks across
        many updates the Kalman gain on the next observation also
        shrinks, so a converged ``θ`` becomes increasingly resistant
        to single noisy fits. This is what makes the candidate goal
        in map frame actually converge instead of swinging on every
        resync event.

        Measurement model: H = [0, 0, 1], R = theta_meas_std². The
        gain ``K = P[:, 2] / (P[2,2] + R)`` is a 3-vector. Current
        production tuning keeps θ decorrelated from GPS position so this
        update normally affects θ only, while still preserving the
        general scalar Kalman form if covariance is reintroduced later.
        """
        R_theta = float(theta_meas_std) ** 2
        innovation = wrap_pi(float(theta_obs) - self.x[2])
        S = float(self.P[2, 2]) + R_theta
        if S <= 0.0:
            return False
        k0 = float(self.P[0, 2]) / S
        k1 = float(self.P[1, 2]) / S
        k2 = float(self.P[2, 2]) / S
        self.x[0] += k0 * innovation
        self.x[1] += k1 * innovation
        self.x[2] = wrap_pi(self.x[2] + k2 * innovation)

        # H = [0,0,1] selects row 2: (I - K H) P == P - outer(K, P[2,:]).
        # Hand-code the 3x3 scalar update to avoid allocating on each GPS
        # heading measurement.
        p20 = float(self.P[2, 0])
        p21 = float(self.P[2, 1])
        p22 = float(self.P[2, 2])
        self.P[0, 0] -= k0 * p20
        self.P[0, 1] -= k0 * p21
        self.P[0, 2] -= k0 * p22
        self.P[1, 0] -= k1 * p20
        self.P[1, 1] -= k1 * p21
        self.P[1, 2] -= k1 * p22
        self.P[2, 0] -= k2 * p20
        self.P[2, 1] -= k2 * p21
        self.P[2, 2] -= k2 * p22

        p01 = 0.5 * (self.P[0, 1] + self.P[1, 0])
        p02 = 0.5 * (self.P[0, 2] + self.P[2, 0])
        p12 = 0.5 * (self.P[1, 2] + self.P[2, 1])
        self.P[0, 1] = self.P[1, 0] = p01
        self.P[0, 2] = self.P[2, 0] = p02
        self.P[1, 2] = self.P[2, 1] = p12
        self.update_count += 1
        return True

    # ── Properties ─────────────────────────────────────────────────
    @property
    def pos_xy(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def theta(self) -> float:
        return float(self.x[2])

    @property
    def theta_std_rad(self) -> float:
        return math.sqrt(max(float(self.P[2, 2]), 0.0))

    @property
    def pos_std(self) -> Tuple[float, float]:
        return (
            math.sqrt(max(float(self.P[0, 0]), 0.0)),
            math.sqrt(max(float(self.P[1, 1]), 0.0)),
        )


# ── Closed-form heading fits (free functions; node-agnostic) ────────
HistoryEntry = Tuple[float, Tuple[float, float], Tuple[float, float]]
"""``(timestamp_s, gps_xy_world_meters, odom_xy_meters)``."""


def _closed_form_fit_iter(
    anchor: HistoryEntry,
    rest: Iterable[HistoryEntry],
    min_baseline: float,
) -> Tuple[Optional[float], float]:
    """Weighted circular mean of ``atan2(Δgps) − atan2(Δodom)`` over the
    given pairs, anchored on ``anchor``. Pairs with magnitude ratio
    outside ``[1/HEADING_FIT_MAGRATIO_MAX, HEADING_FIT_MAGRATIO_MAX]``
    are dropped (almost always spoofer-pinned or jam-dropout samples).
    Returns ``(theta, max_baseline)`` or ``(None, 0.0)``.

    Single-pass over an iterable so callers don't have to materialize
    a list first — important when the input is a 400-element deque
    that ``bootstrap_theta`` and ``closed_form_theta_window`` are
    each called on every GPS tick.
    """
    _, g0, o0 = anchor
    cos_sum = 0.0
    sin_sum = 0.0
    w_sum = 0.0
    max_b = 0.0
    inv_ratio_max = 1.0 / HEADING_FIT_MAGRATIO_MAX
    for _, gi, oi in rest:
        bdx = oi[0] - o0[0]
        bdy = oi[1] - o0[1]
        bl = math.hypot(bdx, bdy)
        if bl < min_baseline:
            continue
        gdx = gi[0] - g0[0]
        gdy = gi[1] - g0[1]
        gl = math.hypot(gdx, gdy)
        ratio = gl / bl if bl > 1e-9 else 0.0
        if ratio < inv_ratio_max or ratio > HEADING_FIT_MAGRATIO_MAX:
            continue
        theta = math.atan2(gdy, gdx) - math.atan2(bdy, bdx)
        cos_sum += bl * math.cos(theta)
        sin_sum += bl * math.sin(theta)
        w_sum += bl
        if bl > max_b:
            max_b = bl
    if w_sum == 0.0:
        return None, 0.0
    return math.atan2(sin_sum, cos_sum), max_b


def bootstrap_theta(
    history: Iterable[HistoryEntry],
    min_baseline: float = 1.5,
    window: Optional[int] = None,
) -> Tuple[Optional[float], float]:
    """Sim ``_bootstrap_theta``. Closed-form heading-offset estimate
    from accumulated GPS+odom pairs, weighted circular mean.

    Anchor choice (controls drift contamination):
      * ``window=None`` — anchor on the very first sample of the
        whole history. Original behaviour. With encoder yaw bias
        active, late samples have accumulated drift; the fit
        averages over baselines that all carry that drift, biasing
        θ toward (true_heading − ⟨drift⟩).
      * ``window=N`` (sliding anchor) — anchor on the OLDEST sample
        within the trailing N entries. Reduces the time-span
        between anchor and recent samples, so each pair carries
        less relative drift. Mirrors the same N-sample sliding
        window used by ``closed_form_theta_window``.

    Accepts any iterable (deque, list, tuple, generator). Iterates
    the sequence in a single pass via ``iter()`` / ``itertools.islice``
    — no list copy required.
    """
    n = getattr(history, "__len__", None)
    if n is not None and n() < 4:
        return None, 0.0
    if window is not None and n is not None and n() > window:
        # Use the same sliding-anchor mechanic as
        # closed_form_theta_window: anchor on the oldest sample in
        # the trailing N.
        start = n() - window
        it = itertools.islice(iter(history), start, None)
    else:
        it = iter(history)
    try:
        anchor = next(it)
    except StopIteration:
        return None, 0.0
    return _closed_form_fit_iter(anchor, it, min_baseline)


def closed_form_theta_window(
    history: Sequence[HistoryEntry],
    n_samples: int,
    min_baseline: float = 2.0,
) -> Tuple[Optional[float], float]:
    """Sim ``_closed_form_theta_window`` (lines 1641-1679).

    Identical to ``bootstrap_theta`` but anchored on the oldest sample
    of the trailing ``n_samples`` window. The sliding anchor means a
    multipath-corrupted *first* fix doesn't poison the post-bootstrap
    re-fit forever.

    Accepts any sized sequence — including ``collections.deque`` — and
    uses ``itertools.islice`` to window without materializing an
    intermediate list. (Slicing a deque directly raises ``TypeError``;
    ``islice`` works on any iterable.)
    """
    n = len(history)
    if n < 4:
        return None, 0.0
    start = max(0, n - n_samples)
    it = itertools.islice(iter(history), start, None)
    try:
        anchor = next(it)
    except StopIteration:
        return None, 0.0
    return _closed_form_fit_iter(anchor, it, min_baseline)


__all__ = [
    "GpsEkf",
    "HistoryEntry",
    "bootstrap_theta",
    "closed_form_theta_window",
    "wrap_pi",
    "EKF_GPS_SIGMA",
    "EKF_GATE_CHI2",
    "EKF_REJ_STREAK_RESET",
    "EKF_POS_VAR_FLOOR",
    "HEADING_FIT_MAGRATIO_MAX",
]
