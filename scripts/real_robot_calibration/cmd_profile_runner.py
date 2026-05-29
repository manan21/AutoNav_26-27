#!/usr/bin/env python3
"""Publish scripted /cmd_vel calibration profiles on the real robot."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

from profile_info import (
    MPH_TO_MPS,
    duration_seconds,
    get_profile,
    iter_segments,
    load_config,
    max_abs_speed_mps,
    segment_velocity,
    validate_profile,
)


class AutoModeLost(RuntimeError):
    pass


def describe_profile(profile_name: str, profile: dict[str, Any]) -> None:
    duration = duration_seconds(profile)
    print(f"Profile: {profile_name}")
    print(f"Description: {profile.get('description', '')}")
    print(f"Command mode: {profile.get('command_mode', 'none')}")
    print(f"Wait for AUTO: {bool(profile.get('wait_for_auto', False))}")
    print(f"Expected duration: {'manual stop' if duration is None else f'{duration:.1f} s'}")
    print(f"Max command speed: {max_abs_speed_mps(profile):.3f} m/s ({max_abs_speed_mps(profile) / MPH_TO_MPS:.2f} mph)")
    if profile.get("operator_notes"):
        print(f"Operator notes: {profile['operator_notes']}")
    for idx, segment in enumerate(iter_segments(profile), start=1):
        linear, angular = segment_velocity(segment)
        print(
            f"  {idx:02d}. {segment.get('label', 'segment')}: "
            f"{float(segment.get('duration_s', 0.0)):.1f}s, "
            f"linear={linear:.3f} m/s ({linear / MPH_TO_MPS:.2f} mph), "
            f"angular={angular:.3f} rad/s"
        )


def run_ros_profile(args: argparse.Namespace, profile_name: str, profile: dict[str, Any], defaults: dict[str, Any]) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    reliable_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )

    class CmdRunner(Node):
        def __init__(self) -> None:
            super().__init__("real_robot_calibration_cmd_runner")
            self.auto_mode: bool | None = None
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Bool, "/autonomous_mode", self._on_auto, reliable_qos)

        def _on_auto(self, msg: Bool) -> None:
            self.auto_mode = bool(msg.data)

        def publish_cmd(self, linear: float, angular: float) -> None:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.pub.publish(msg)

        def publish_zero_for(self, duration_s: float, rate_hz: float) -> None:
            self.publish_for("zero_hold", duration_s, 0.0, 0.0, rate_hz, require_auto=False)

        def wait_for_auto(self, timeout_s: float) -> None:
            deadline = time.monotonic() + timeout_s
            last_print = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.auto_mode is True:
                    print("AUTO observed on /autonomous_mode. Starting scripted commands.")
                    return
                now = time.monotonic()
                if now - last_print > 2.0:
                    print("Waiting for /autonomous_mode=true. Toggle AUTO with Xbox X when safe.")
                    last_print = now
            raise TimeoutError("Timed out waiting for /autonomous_mode=true; no motion commanded")

        def publish_for(
            self,
            label: str,
            duration_s: float,
            linear: float,
            angular: float,
            rate_hz: float,
            *,
            require_auto: bool,
        ) -> None:
            period_s = 1.0 / max(rate_hz, 1.0)
            end_time = time.monotonic() + duration_s
            next_print = 0.0
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.0)
                if require_auto and self.auto_mode is False:
                    raise AutoModeLost("AUTO turned false; stopping scripted profile")
                self.publish_cmd(linear, angular)
                now = time.monotonic()
                if now >= next_print:
                    remaining = max(0.0, end_time - now)
                    print(
                        f"{label}: remaining={remaining:.1f}s "
                        f"linear={linear:.3f} m/s ({linear / MPH_TO_MPS:.2f} mph) "
                        f"angular={angular:.3f} rad/s",
                        flush=True,
                    )
                    next_print = now + 2.0
                time.sleep(period_s)

    rclpy.init()
    node = CmdRunner()
    rate_hz = float(args.command_rate_hz or defaults.get("command_rate_hz", 20.0))
    zero_hold_s = float(args.zero_hold_s or defaults.get("zero_hold_s", 2.0))
    auto_timeout_s = float(args.auto_timeout_s or defaults.get("auto_timeout_s", 60.0))
    require_auto = bool(profile.get("wait_for_auto", False))

    try:
        print()
        print("VIDEO SYNC: say the profile/run name now, then wave in front of camera and lidar.")
        print(f"RUNNING PROFILE: {profile_name}")
        print()
        node.publish_zero_for(zero_hold_s, rate_hz)
        if require_auto:
            node.wait_for_auto(auto_timeout_s)
        for segment in iter_segments(profile):
            label = str(segment.get("label", "segment"))
            duration_s = float(segment["duration_s"])
            linear, angular = segment_velocity(segment)
            node.publish_for(label, duration_s, linear, angular, rate_hz, require_auto=require_auto)
        print("Scripted profile complete. Publishing stop commands.")
        node.publish_zero_for(zero_hold_s, rate_hz)
        return 0
    except KeyboardInterrupt:
        print("Interrupted. Publishing stop commands.")
        node.publish_zero_for(zero_hold_s, rate_hz)
        return 130
    except AutoModeLost as exc:
        print(str(exc), file=sys.stderr)
        node.publish_zero_for(zero_hold_s, rate_hz)
        return 10
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        node.publish_zero_for(zero_hold_s, rate_hz)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profiles", type=Path, default=Path(__file__).with_name("profiles.yaml"))
    parser.add_argument("--profile", required=True)
    parser.add_argument("--allow-high-speed", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--command-rate-hz", type=float)
    parser.add_argument("--zero-hold-s", type=float)
    parser.add_argument("--auto-timeout-s", type=float)
    args = parser.parse_args()

    config = load_config(args.profiles)
    validate_profile(args.profile, config)
    profile = get_profile(config, args.profile)
    defaults = config.get("defaults", {})

    describe_profile(args.profile, profile)
    if profile.get("requires_allow_high_speed") and not args.allow_high_speed:
        print("ERROR: this profile requires --allow-high-speed", file=sys.stderr)
        return 2

    if profile.get("command_mode") != "scripted":
        print("No scripted command to run for this profile.")
        return 0

    if args.dry_run:
        print("Dry run only; no ROS commands published.")
        return 0

    return run_ros_profile(args, args.profile, profile, defaults)


if __name__ == "__main__":
    sys.exit(main())
