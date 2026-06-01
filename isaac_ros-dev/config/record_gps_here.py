#!/usr/bin/env python3
"""record_gps_here.py — sample /gps_fix, average, persist as a waypoint.

Implements the recording half of the GI / GR mission pair driven by
run_mission.sh:

  * GI fires this script. We sample /gps_fix N times (default 10),
    average lat / lon, and write a single "lat lon" line to the path
    given on the command line.
  * GR (handled in run_mission.sh) reads that file and dispatches
    send_GPS_waypoint.sh to return the robot to the recorded spot.

QoS mirrors mission_precheck.py — /gps_fix is BEST_EFFORT KEEP_LAST,
which is what every other consumer of that topic in this repo uses.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix


SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)


class GpsRecorder(Node):
    def __init__(self, samples_needed: int) -> None:
        super().__init__("record_gps_here")
        self.samples_needed = samples_needed
        self.samples: list[tuple[float, float]] = []
        self.create_subscription(NavSatFix, "/gps_fix", self._on_gps, SENSOR_QOS)

    def _on_gps(self, msg: NavSatFix) -> None:
        # NavSatStatus.STATUS_NO_FIX is -1; anything < 0 is unusable.
        if msg.status.status < 0:
            return
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        # NaN guard — a momentary NaN slipping into the average ruins it.
        if lat != lat or lon != lon:
            return
        if len(self.samples) >= self.samples_needed:
            return
        self.samples.append((lat, lon))
        self.get_logger().info(
            f"GI sample {len(self.samples)}/{self.samples_needed}: "
            f"lat={lat:.7f} lon={lon:.7f}"
        )

    def done(self) -> bool:
        return len(self.samples) >= self.samples_needed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("output", help="path to write the averaged 'lat lon' line")
    parser.add_argument("--samples", type=int, default=10,
                        help="number of valid /gps_fix samples to average (default: 10)")
    args = parser.parse_args()

    rclpy.init()
    node = GpsRecorder(args.samples)

    # No timeout — the mission precheck has already gated on /gps_fix being
    # live, and aborting GI partway through (with a half-finished average,
    # or none at all) would silently corrupt the GR return point. If GPS
    # stops streaming, prefer hanging here so the operator notices.
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = list(node.samples)
    node.destroy_node()
    rclpy.shutdown()

    n = len(samples)
    if n < args.samples:
        # Reachable only if rclpy was shut down (Ctrl+C, SIGTERM). Treat as
        # an abort, not a "use what we got" — a partial average is worse
        # than no GR target at all.
        print(f"ERROR: GI interrupted with only {n}/{args.samples} samples — "
              f"not writing waypoint", file=sys.stderr)
        return 1

    lat_avg = sum(s[0] for s in samples) / n
    lon_avg = sum(s[1] for s in samples) / n

    with open(args.output, "w") as f:
        f.write(f"{lat_avg:.7f} {lon_avg:.7f}\n")
    print(f"GI recorded: lat={lat_avg:.7f} lon={lon_avg:.7f} "
          f"({n} samples) -> {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
