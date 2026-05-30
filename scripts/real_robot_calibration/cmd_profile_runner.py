#!/usr/bin/env python3
"""Publish scripted /cmd_vel calibration profiles on the real robot."""

from __future__ import annotations

import argparse
import csv
import math
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


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


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
        duration_s = float(segment.get("duration_s", 0.0))
        if "target_distance_m" in segment:
            target = float(segment["target_distance_m"])
            odom_topic = str(segment.get("distance_odom_topic", profile.get("distance_odom_topic", "/odom")))
            timing = f"target={target:.2f}m, max={duration_s:.1f}s, odom={odom_topic}"
        else:
            timing = f"{duration_s:.1f}s"
        print(
            f"  {idx:02d}. {segment.get('label', 'segment')}: "
            f"{timing}, "
            f"linear={linear:.3f} m/s ({linear / MPH_TO_MPS:.2f} mph), "
            f"angular={angular:.3f} rad/s"
        )


def run_ros_profile(args: argparse.Namespace, profile_name: str, profile: dict[str, Any], defaults: dict[str, Any]) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
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
            self.latest_odom: dict[str, dict[str, float]] = {}
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Bool, "/autonomous_mode", self._on_auto, reliable_qos)
            self.create_subscription(Odometry, "/odom", lambda msg: self._on_odom("/odom", msg), reliable_qos)
            self.create_subscription(Odometry, "/local_ekf/odom", lambda msg: self._on_odom("/local_ekf/odom", msg), reliable_qos)

        def _on_auto(self, msg: Bool) -> None:
            self.auto_mode = bool(msg.data)

        def _on_odom(self, topic: str, msg: Odometry) -> None:
            pose = msg.pose.pose
            self.latest_odom[topic] = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw_from_quaternion(pose.orientation),
                "stamp_s": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                "received_s": time.monotonic(),
            }

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

        def wait_for_odom(self, odom_topic: str, timeout_s: float = 5.0) -> dict[str, float]:
            deadline = time.monotonic() + timeout_s
            last_print = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                sample = self.latest_odom.get(odom_topic)
                if sample is not None:
                    return sample.copy()
                now = time.monotonic()
                if now - last_print > 2.0:
                    print(f"Waiting for odometry on {odom_topic}...")
                    last_print = now
            raise TimeoutError(f"Timed out waiting for odometry on {odom_topic}")

        def displacement_metrics(
            self,
            *,
            label: str,
            odom_topic: str,
            start: dict[str, float],
            current: dict[str, float],
            linear: float,
            angular: float,
            target_distance_m: float,
            max_duration_s: float,
            elapsed_s: float,
            reached: bool,
        ) -> dict[str, float | str | bool]:
            dx = current["x"] - start["x"]
            dy = current["y"] - start["y"]
            start_yaw = start["yaw"]
            cos_yaw = math.cos(start_yaw)
            sin_yaw = math.sin(start_yaw)
            raw_forward = dx * cos_yaw + dy * sin_yaw
            direction = 1.0 if linear >= 0.0 else -1.0
            forward_m = raw_forward * direction
            lateral_m = -dx * sin_yaw + dy * cos_yaw
            heading_drift_rad = normalize_angle(current["yaw"] - start_yaw)
            return {
                "segment": label,
                "odom_topic": odom_topic,
                "reached": reached,
                "target_distance_m": target_distance_m,
                "max_duration_s": max_duration_s,
                "elapsed_s": elapsed_s,
                "command_linear_mps": linear,
                "command_angular_radps": angular,
                "forward_m": forward_m,
                "lateral_m": lateral_m,
                "euclidean_m": math.hypot(dx, dy),
                "heading_drift_rad": heading_drift_rad,
                "heading_drift_deg": math.degrees(heading_drift_rad),
                "overshoot_m": forward_m - target_distance_m,
                "start_x": start["x"],
                "start_y": start["y"],
                "start_yaw_rad": start_yaw,
                "end_x": current["x"],
                "end_y": current["y"],
                "end_yaw_rad": current["yaw"],
                "odom_elapsed_s": current["stamp_s"] - start["stamp_s"],
            }

        def publish_until_distance(
            self,
            label: str,
            target_distance_m: float,
            max_duration_s: float,
            linear: float,
            angular: float,
            rate_hz: float,
            *,
            require_auto: bool,
            odom_topic: str,
        ) -> dict[str, float | str | bool]:
            period_s = 1.0 / max(rate_hz, 1.0)
            start = self.wait_for_odom(odom_topic)
            started_s = time.monotonic()
            end_time = started_s + max_duration_s
            next_print = 0.0
            current = start
            reached = False
            metrics = self.displacement_metrics(
                label=label,
                odom_topic=odom_topic,
                start=start,
                current=current,
                linear=linear,
                angular=angular,
                target_distance_m=target_distance_m,
                max_duration_s=max_duration_s,
                elapsed_s=0.0,
                reached=False,
            )
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.0)
                if require_auto and self.auto_mode is False:
                    raise AutoModeLost("AUTO turned false; stopping scripted profile")
                current = self.latest_odom.get(odom_topic, current)
                elapsed_s = time.monotonic() - started_s
                metrics = self.displacement_metrics(
                    label=label,
                    odom_topic=odom_topic,
                    start=start,
                    current=current,
                    linear=linear,
                    angular=angular,
                    target_distance_m=target_distance_m,
                    max_duration_s=max_duration_s,
                    elapsed_s=elapsed_s,
                    reached=False,
                )
                if float(metrics["forward_m"]) >= target_distance_m:
                    reached = True
                    metrics["reached"] = True
                    break
                self.publish_cmd(linear, angular)
                now = time.monotonic()
                if now >= next_print:
                    print(
                        f"{label}: forward={float(metrics['forward_m']):.3f}/{target_distance_m:.3f}m "
                        f"lateral={float(metrics['lateral_m']):+.3f}m "
                        f"heading={float(metrics['heading_drift_deg']):+.2f}deg "
                        f"linear={linear:.3f} m/s ({linear / MPH_TO_MPS:.2f} mph)",
                        flush=True,
                    )
                    next_print = now + 1.0
                time.sleep(period_s)
            self.publish_cmd(0.0, 0.0)
            metrics["reached"] = reached
            print(
                f"{label} complete: reached={reached} "
                f"forward={float(metrics['forward_m']):.3f}m "
                f"lateral={float(metrics['lateral_m']):+.3f}m "
                f"heading={float(metrics['heading_drift_deg']):+.2f}deg "
                f"overshoot={float(metrics['overshoot_m']):+.3f}m",
                flush=True,
            )
            if not reached:
                raise TimeoutError(f"{label}: target distance {target_distance_m:.3f}m not reached within {max_duration_s:.1f}s")
            return metrics

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
    metrics_file = Path(args.metrics_file) if args.metrics_file else None
    metrics_handle = None
    metrics_writer = None
    metric_fields = [
        "segment",
        "odom_topic",
        "reached",
        "target_distance_m",
        "max_duration_s",
        "elapsed_s",
        "command_linear_mps",
        "command_angular_radps",
        "forward_m",
        "lateral_m",
        "euclidean_m",
        "heading_drift_rad",
        "heading_drift_deg",
        "overshoot_m",
        "start_x",
        "start_y",
        "start_yaw_rad",
        "end_x",
        "end_y",
        "end_yaw_rad",
        "odom_elapsed_s",
    ]
    if metrics_file is not None:
        metrics_file.parent.mkdir(parents=True, exist_ok=True)
        metrics_handle = metrics_file.open("w", encoding="utf-8", newline="")
        metrics_writer = csv.DictWriter(metrics_handle, fieldnames=metric_fields)
        metrics_writer.writeheader()

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
            if "target_distance_m" in segment:
                odom_topic = str(segment.get("distance_odom_topic", profile.get("distance_odom_topic", "/odom")))
                metrics = node.publish_until_distance(
                    label,
                    float(segment["target_distance_m"]),
                    duration_s,
                    linear,
                    angular,
                    rate_hz,
                    require_auto=require_auto,
                    odom_topic=odom_topic,
                )
                if metrics_writer is not None:
                    metrics_writer.writerow(metrics)
                    metrics_handle.flush()
            else:
                node.publish_for(label, duration_s, linear, angular, rate_hz, require_auto=require_auto)
        print("Scripted profile complete. Publishing stop commands.")
        node.publish_zero_for(zero_hold_s, rate_hz)
        if metrics_file is not None:
            print(f"Distance metrics: {metrics_file}")
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
        if metrics_handle is not None:
            metrics_handle.close()
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
    parser.add_argument("--metrics-file", type=Path)
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
