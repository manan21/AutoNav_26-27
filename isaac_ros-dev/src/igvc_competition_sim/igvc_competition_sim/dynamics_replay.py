#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from .dynamics_config import (
    default_replay_profiles_path,
    load_replay_profiles,
    segment_angular_radps,
    segment_linear_mps,
)

ROS_IMPORT_ERROR: Exception | None = None
try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.exceptions import RCLError
    from rclpy.node import Node
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    ROS_IMPORT_ERROR = exc
    rclpy = None
    Twist = None
    ExternalShutdownException = Exception
    RCLError = Exception
    Node = object


class DynamicsReplay(Node):
    def __init__(self,
                 profile_name: str,
                 profiles_path: str,
                 command_topic: str,
                 rate_hz: float) -> None:
        super().__init__("igvc_dynamics_replay")
        self.profile_name = profile_name
        self.config = load_replay_profiles(profiles_path)
        profiles = self.config.get("profiles", {})
        if profile_name not in profiles:
            available = ", ".join(sorted(str(name) for name in profiles))
            raise RuntimeError(
                "unknown replay profile %s; available: %s"
                % (profile_name, available))
        self.profile = profiles[profile_name]
        defaults = self.config.get("defaults", {})
        self.rate_hz = rate_hz or float(defaults.get("command_rate_hz", 20.0))
        self.segments: list[dict[str, Any]] = list(
            self.profile.get("segments", []))
        if not self.segments:
            raise RuntimeError("profile %s has no segments" % profile_name)
        self.publisher = self.create_publisher(Twist, command_topic, 10)
        self.segment_index = 0
        self.segment_started_mono = time.monotonic()
        self.done = False
        self.timer = self.create_timer(
            1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            "replaying %s to %s (%d segments)"
            % (profile_name, command_topic, len(self.segments)))

    def _tick(self) -> None:
        if self.done:
            self._publish(0.0, 0.0)
            return
        now = time.monotonic()
        segment = self.segments[self.segment_index]
        elapsed = now - self.segment_started_mono
        duration = float(segment.get("duration_s", 0.0))
        if elapsed >= duration:
            self.segment_index += 1
            if self.segment_index >= len(self.segments):
                self.done = True
                self._publish(0.0, 0.0)
                self.get_logger().info("profile %s complete" % self.profile_name)
                return
            self.segment_started_mono = now
            segment = self.segments[self.segment_index]
            label = str(segment.get("label", self.segment_index))
            self.get_logger().info("segment %s" % label)
        self._publish(segment_linear_mps(segment), segment_angular_radps(segment))

    def _publish(self, linear_mps: float, angular_radps: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_mps)
        msg.angular.z = float(angular_radps)
        self.publisher.publish(msg)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", nargs="?", help="Replay profile name.")
    parser.add_argument(
        "--profiles",
        default=str(default_replay_profiles_path()),
        help="Replay profile YAML path.",
    )
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--rate-hz", type=float, default=0.0)
    parser.add_argument("--list", action="store_true")
    raw_args = sys.argv[1:] if argv is None else argv
    args, ros_args = parser.parse_known_args(raw_args)

    if args.list:
        config = load_replay_profiles(args.profiles)
        for name in sorted(config.get("profiles", {})):
            print(name)
        return 0
    if not args.profile:
        parser.error("profile is required unless --list is used")
    if ROS_IMPORT_ERROR is not None:
        raise SystemExit(
            "igvc_dynamics_replay must run in a sourced ROS 2 Humble environment"
        ) from ROS_IMPORT_ERROR

    rclpy.init(args=([sys.argv[0]] + ros_args) if argv is None else ros_args)
    node = DynamicsReplay(args.profile, args.profiles, args.topic, args.rate_hz)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):
            node._publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
