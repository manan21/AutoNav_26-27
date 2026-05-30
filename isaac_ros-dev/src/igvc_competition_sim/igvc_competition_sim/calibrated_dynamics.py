#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys

from .dynamics_config import as_bool, as_float, clamp, default_calibration_path, load_config

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.exceptions import RCLError
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_calibrated_dynamics must run in a sourced ROS 2 Humble environment"
    ) from exc


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _first_order(current: float, target: float, dt: float, tau: float) -> float:
    if tau <= 1e-6:
        return target
    alpha = 1.0 - math.exp(-max(0.0, dt) / tau)
    return current + alpha * (target - current)


def _limit_delta(current: float, target: float, max_delta: float) -> float:
    if max_delta <= 0.0:
        return target
    return current + clamp(target - current, -max_delta, max_delta)


class CalibratedDynamics(Node):
    def __init__(self) -> None:
        super().__init__("igvc_calibrated_dynamics")
        self.declare_parameter("calibration_config", str(default_calibration_path()))
        config_path = str(self.get_parameter("calibration_config").value)
        self.config = load_config(config_path)
        model = self.config.get("dynamics_model", {})

        self.declare_parameter("enabled", bool(model.get("enabled", True)))
        self.enabled = as_bool(self.get_parameter("enabled").value, True)

        self.input_topic = str(model.get("input_topic", "/cmd_vel"))
        self.output_topic = str(model.get("output_topic", "/cmd_vel_gazebo"))
        self.state_topic = str(
            model.get("state_topic", "/igvc_sim/dynamics_state"))
        self.calibration_topic = str(
            model.get("calibration_topic", "/igvc_sim/dynamics_calibration"))

        self.update_rate_hz = as_float(model.get("update_rate_hz"), 50.0)
        self.command_timeout_s = as_float(model.get("command_timeout_s"), 0.40)
        self.command_latency_s = as_float(model.get("command_latency_s"), 0.08)
        self.max_linear_speed_mps = as_float(
            model.get("max_linear_speed_mps"), 0.50)
        self.max_angular_speed_radps = as_float(
            model.get("max_angular_speed_radps"), 1.00)
        self.linear_gain = as_float(model.get("linear_gain"), 1.00)
        self.angular_gain = as_float(model.get("angular_gain"), 1.00)
        self.positive_angular_gain = as_float(
            model.get("positive_angular_gain"), 1.00)
        self.negative_angular_gain = as_float(
            model.get("negative_angular_gain"), 1.00)
        self.linear_tau_s = as_float(model.get("linear_time_constant_s"), 0.20)
        self.angular_tau_s = as_float(
            model.get("angular_time_constant_s"), 0.18)
        self.max_linear_accel = as_float(
            model.get("max_linear_acceleration_mps2"), 1.00)
        self.max_angular_accel = as_float(
            model.get("max_angular_acceleration_radps2"), 2.00)
        self.linear_deadband = as_float(model.get("linear_deadband_mps"), 0.0)
        self.angular_deadband = as_float(
            model.get("angular_deadband_radps"), 0.0)
        self.flat_ground_yaw_bias = as_float(
            model.get("flat_ground_yaw_bias_radps"), 0.0)

        self.pending_commands: list[tuple[float, float, float]] = []
        self.target_v = 0.0
        self.target_w = 0.0
        self.applied_v = 0.0
        self.applied_w = 0.0
        self.last_input_s = -math.inf
        self.last_step_s: float | None = None
        self.last_state_publish_s = -math.inf

        self.cmd_sub = self.create_subscription(
            Twist, self.input_topic, self._cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        latch_qos = QoSProfile(depth=1)
        latch_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.calibration_pub = self.create_publisher(
            String, self.calibration_topic, latch_qos)

        period = 1.0 / max(1.0, self.update_rate_hz)
        self.timer = self.create_timer(period, self._step)
        self._publish_calibration()
        self.get_logger().info(
            "routing %s -> %s with calibrated dynamics enabled=%s"
            % (self.input_topic, self.output_topic, self.enabled))

    def _cmd_callback(self, msg: Twist) -> None:
        now_s = _stamp_to_float(self.get_clock().now().to_msg())
        linear = clamp(float(msg.linear.x),
                       -self.max_linear_speed_mps,
                       self.max_linear_speed_mps)
        angular = clamp(float(msg.angular.z),
                        -self.max_angular_speed_radps,
                        self.max_angular_speed_radps)
        latency_s = max(0.0, self.command_latency_s) if self.enabled else 0.0
        self.pending_commands.append((now_s + latency_s, linear, angular))
        self.last_input_s = now_s

    def _step(self) -> None:
        stamp = self.get_clock().now().to_msg()
        now_s = _stamp_to_float(stamp)
        if self.last_step_s is None:
            self.last_step_s = now_s
        dt = max(0.0, min(0.10, now_s - self.last_step_s))
        self.last_step_s = now_s

        while self.pending_commands and self.pending_commands[0][0] <= now_s:
            _, self.target_v, self.target_w = self.pending_commands.pop(0)
        if now_s - self.last_input_s > self.command_timeout_s:
            self.target_v = 0.0
            self.target_w = 0.0

        if self.enabled:
            out_v, out_w = self._filtered_command(dt)
        else:
            out_v, out_w = self.target_v, self.target_w
            self.applied_v = out_v
            self.applied_w = out_w

        msg = Twist()
        msg.linear.x = out_v
        msg.angular.z = out_w
        self.cmd_pub.publish(msg)
        if now_s - self.last_state_publish_s >= 0.20:
            self.last_state_publish_s = now_s
            self._publish_state(now_s)

    def _filtered_command(self, dt: float) -> tuple[float, float]:
        target_v = 0.0 if abs(self.target_v) < self.linear_deadband else self.target_v
        target_w = 0.0 if abs(self.target_w) < self.angular_deadband else self.target_w

        target_v *= self.linear_gain
        if target_w > 0.0:
            target_w *= self.angular_gain * self.positive_angular_gain
        elif target_w < 0.0:
            target_w *= self.angular_gain * self.negative_angular_gain
        if abs(target_v) > 1e-6:
            target_w += self.flat_ground_yaw_bias

        target_v = clamp(target_v, -self.max_linear_speed_mps,
                         self.max_linear_speed_mps)
        target_w = clamp(target_w, -self.max_angular_speed_radps,
                         self.max_angular_speed_radps)

        filtered_v = _first_order(
            self.applied_v, target_v, dt, self.linear_tau_s)
        filtered_w = _first_order(
            self.applied_w, target_w, dt, self.angular_tau_s)
        self.applied_v = _limit_delta(
            self.applied_v, filtered_v, self.max_linear_accel * dt)
        self.applied_w = _limit_delta(
            self.applied_w, filtered_w, self.max_angular_accel * dt)
        return self.applied_v, self.applied_w

    def _publish_calibration(self) -> None:
        payload = {
            "enabled": self.enabled,
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "command_latency_s": self.command_latency_s,
            "linear_time_constant_s": self.linear_tau_s,
            "angular_time_constant_s": self.angular_tau_s,
            "linear_gain": self.linear_gain,
            "angular_gain": self.angular_gain,
            "positive_angular_gain": self.positive_angular_gain,
            "negative_angular_gain": self.negative_angular_gain,
            "flat_ground_yaw_bias_radps": self.flat_ground_yaw_bias,
            "straight_lateral_drift_policy": (
                "excluded_from_default_flat_ground_tuning"
            ),
        }
        self.calibration_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_state(self, now_s: float) -> None:
        payload = {
            "stamp_s": now_s,
            "enabled": self.enabled,
            "target_linear_mps": self.target_v,
            "target_angular_radps": self.target_w,
            "applied_linear_mps": self.applied_v,
            "applied_angular_radps": self.applied_w,
            "pending_commands": len(self.pending_commands),
        }
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=sys.argv if argv is None else argv)
    node = CalibratedDynamics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
