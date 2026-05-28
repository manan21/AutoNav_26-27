#!/usr/bin/env python3
"""GPS-waypoint mission pre-check — green-light gate for run_mission.sh.

Exits 0 only when every signal that gps_handler / nav2 need to drive a
GA leg is live:

  * /autonomous_mode      == True   (joystick won't fight nav2)
  * /gps_fix              status >= 0 and arriving
  * /local_ekf/odom       arriving (local EKF up — owns odom→base_link
                          TF, is the predict source gps_handler reads)
  * /gps_waypoint/debug   arriving (gps_handler_node up)
  * TF map -> base_link   resolvable (robot is localized)
  * /navigate_to_waypoint action server reachable

Exits 1 on timeout, 2 on import / argparse error.

Mirrors the topic set that /tmp/autonav_monitor.py watched during
outdoor Phase C tests — that monitor is how we noticed which signals
have to be green before the first GA leg should be dispatched.

Note: previously gated on /global_ekf/odom. That topic is published by
ekf_global (a parallel state estimator) and consumed only by HUD/test
logging and the closed navsat_transform↔ekf_global fusion loop — no
NAV2 component or gps_handler subscriber. The launch stack supports
enable_gps_fusion:=false which disables ekf_global entirely while
keeping the mission fully functional. Gating on it created a startup
hang whenever ekf_global was disabled or slow to come up, with no
operational reason. /local_ekf/odom is the load-bearing odom source.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)

from autonav_interfaces.action import NavigateToWaypoint
from geometry_msgs.msg import Twist  # noqa: F401  (kept for future check additions)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)
RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class PreCheck(Node):
    def __init__(self) -> None:
        super().__init__("mission_precheck")

        self.auto_mode: bool | None = None
        self.gps_status: int | None = None
        self.gps_seen: bool = False
        self.odom_seen: bool = False
        self.debug_seen: bool = False

        self._tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buf, self, spin_thread=False)

        self._nav_client = ActionClient(self, NavigateToWaypoint, "/navigate_to_waypoint")

        self.create_subscription(Bool, "/autonomous_mode", self._on_auto, RELIABLE_QOS)
        self.create_subscription(NavSatFix, "/gps_fix", self._on_gps, SENSOR_QOS)
        self.create_subscription(Odometry, "/local_ekf/odom", self._on_odom, RELIABLE_QOS)
        self.create_subscription(String, "/gps_waypoint/debug", self._on_debug, RELIABLE_QOS)

    def _on_auto(self, msg: Bool) -> None:
        self.auto_mode = bool(msg.data)

    def _on_gps(self, msg: NavSatFix) -> None:
        self.gps_seen = True
        self.gps_status = int(msg.status.status)

    def _on_odom(self, _msg: Odometry) -> None:
        self.odom_seen = True

    def _on_debug(self, _msg: String) -> None:
        self.debug_seen = True

    def tf_ok(self) -> bool:
        try:
            self._tf_buf.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.0)
            )
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def action_ok(self) -> bool:
        return self._nav_client.server_is_ready()


def report(name: str, ok: bool, detail: str = "") -> str:
    tag = "GREEN" if ok else "WAIT "
    return f"  [{tag}] {name}{(' — ' + detail) if detail else ''}"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="seconds to wait for all signals to green-light (default 60)",
    )
    p.add_argument(
        "--print-period",
        type=float,
        default=2.0,
        help="seconds between progress prints (default 2)",
    )
    p.add_argument(
        "--require-auto",
        action="store_true",
        default=True,
        help="require /autonomous_mode == True (default on)",
    )
    p.add_argument(
        "--skip-auto",
        dest="require_auto",
        action="store_false",
        help="skip the AUTO-mode check (operator will engage AUTO after mission start)",
    )
    args = p.parse_args()

    rclpy.init()
    node = PreCheck()

    deadline = time.monotonic() + args.timeout
    next_print = 0.0
    last_status = ""
    result = 1

    print("mission_precheck: waiting for GPS-waypoint green-light "
          f"(timeout {args.timeout:.0f}s)", flush=True)

    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            auto_ok = (not args.require_auto) or (node.auto_mode is True)
            gps_ok = node.gps_seen and (node.gps_status is not None) and (node.gps_status >= 0)
            odom_ok = node.odom_seen
            debug_ok = node.debug_seen
            tf_ok = node.tf_ok()
            action_ok = node.action_ok()

            checks = [
                ("autonomous_mode == AUTO",
                 auto_ok,
                 "skipped" if not args.require_auto else (
                     "no message yet" if node.auto_mode is None
                     else ("MAN" if node.auto_mode is False else "AUTO"))),
                ("/gps_fix",
                 gps_ok,
                 "no fix yet" if not node.gps_seen
                 else f"status={node.gps_status}"),
                ("/local_ekf/odom",
                 odom_ok,
                 "" if odom_ok else "no message yet"),
                ("/gps_waypoint/debug (gps_handler up)",
                 debug_ok,
                 "" if debug_ok else "no message yet"),
                ("TF map -> base_link",
                 tf_ok,
                 "" if tf_ok else "transform unavailable"),
                ("/navigate_to_waypoint action server",
                 action_ok,
                 "" if action_ok else "not discovered"),
            ]

            now = time.monotonic()
            status = "\n".join(report(name, ok, detail) for name, ok, detail in checks)
            if now >= next_print and status != last_status:
                remaining = max(0.0, deadline - now)
                print(f"\n[{time.strftime('%H:%M:%S')}] "
                      f"pre-check status (T-{remaining:.0f}s):", flush=True)
                print(status, flush=True)
                next_print = now + args.print_period
                last_status = status

            if all(ok for _, ok, _ in checks):
                print("\nmission_precheck: ALL GREEN — clearing mission to start.",
                      flush=True)
                result = 0
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if result != 0:
        print("\nmission_precheck: TIMED OUT — refusing to start mission.", flush=True)
    return result


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nmission_precheck: interrupted — refusing to start mission.",
              flush=True)
        sys.exit(130)
