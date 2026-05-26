#!/usr/bin/env python3
"""Ensure the control node is in autonomous mode before a Nav2 test goal."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class AutonomousModeEnsurer(Node):
    def __init__(self) -> None:
        super().__init__("ensure_autonomous_mode")
        self.latest_mode = None
        self.latest_mode_time = None
        self.auto_sub = self.create_subscription(
            Bool, "/autonomous_mode", self._on_auto_mode, 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)

    def _on_auto_mode(self, msg: Bool) -> None:
        self.latest_mode = bool(msg.data)
        self.latest_mode_time = time.monotonic()

    def spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def wait_for_joy_subscriber(self, timeout_s: float) -> bool:
        return self.spin_until(
            lambda: self.joy_pub.get_subscription_count() > 0, timeout_s)

    def wait_for_mode_sample(self, timeout_s: float, after_time=None) -> bool:
        def have_sample() -> bool:
            if self.latest_mode is None or self.latest_mode_time is None:
                return False
            return after_time is None or self.latest_mode_time > after_time

        return self.spin_until(have_sample, timeout_s)

    def publish_x_press(self) -> None:
        # Publish exactly one X-button press. The physical joystick stream
        # supplies the release. Repeated synthetic presses can interleave
        # with joystick zero messages and create multiple rising edges,
        # toggling AUTO back off.
        press = Joy()
        press.axes = [0.0] * 4
        press.buttons = [0] * 8
        press.buttons[3] = 1
        self.joy_pub.publish(press)
        self.spin_until(lambda: False, 0.10)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Set /autonomous_mode true by publishing X-button pulses if needed.")
    parser.add_argument(
        "--initial-wait", type=float, default=1.0,
        help="seconds to wait for an existing /autonomous_mode sample")
    parser.add_argument(
        "--toggle-wait", type=float, default=5.0,
        help="seconds to wait for /autonomous_mode after each X press")
    parser.add_argument(
        "--joy-wait", type=float, default=5.0,
        help="seconds to wait for a /joy subscriber before toggling")
    parser.add_argument(
        "--max-toggles", type=int, default=2,
        help="maximum X-button pulses before failing")
    parser.add_argument(
        "--settle", type=float, default=0.30,
        help="seconds to wait after AUTO is confirmed before returning")
    args = parser.parse_args()

    rclpy.init()
    node = AutonomousModeEnsurer()
    try:
        node.wait_for_mode_sample(args.initial_wait)
        if node.latest_mode is True:
            node.get_logger().info("Autonomous mode already enabled.")
            return 0

        if not node.wait_for_joy_subscriber(args.joy_wait):
            node.get_logger().error(
                "No /joy subscriber found; cannot toggle autonomous mode.")
            return 2

        for attempt in range(1, args.max_toggles + 1):
            before = node.latest_mode_time
            state_text = (
                "unknown" if node.latest_mode is None
                else ("AUTO" if node.latest_mode else "MANUAL"))
            node.get_logger().info(
                f"Autonomous mode is {state_text}; sending one X press "
                f"{attempt}/{args.max_toggles}.")
            node.publish_x_press()
            node.wait_for_mode_sample(args.toggle_wait, after_time=before)

            if node.latest_mode is True:
                node.get_logger().info("Autonomous mode confirmed enabled.")
                node.spin_until(lambda: False, args.settle)
                return 0

        final_state = (
            "unknown" if node.latest_mode is None
            else ("AUTO" if node.latest_mode else "MANUAL"))
        node.get_logger().error(
            f"Autonomous mode did not become AUTO; final state is {final_state}.")
        return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
