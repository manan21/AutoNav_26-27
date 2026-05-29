#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys

from .course import Course, load_course

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_course_monitor must run in a sourced ROS 2 Humble environment"
    ) from exc


def _stamp_s(node: Node) -> float:
    stamp = node.get_clock().now().to_msg()
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _point_segment_distance(x: float,
                            y: float,
                            start: tuple[float, float],
                            end: tuple[float, float]) -> float:
    ax, ay = start
    bx, by = end
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / denom))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(x - qx, y - qy)


class IgvcCourseMonitor(Node):
    def __init__(self) -> None:
        super().__init__("igvc_course_monitor")
        self.declare_parameter("course_config", "")
        self.declare_parameter("sample_spacing_m", 0.05)
        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        self.robot = self.course.robot
        self.sample_spacing_m = max(
            0.02, float(self.get_parameter("sample_spacing_m").value))

        self.score_pub = self.create_publisher(String, "/igvc_sim/score", 10)
        self.fail_pub = self.create_publisher(Bool, "/igvc_sim/fail", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 20)

        self.last_pose: tuple[float, float, float] | None = None
        self.last_time_s: float | None = None
        self.distance_m = 0.0
        self.speed_check_start_s: float | None = None
        self.speed_check_start_distance_m: float = 0.0
        self.speed_check_end_s: float | None = None
        self.stop_started_s: float | None = None
        self.autonomous = True
        self.failures: list[str] = []
        self.max_speed_mps = 0.0
        self.finish_reached = False
        self.create_timer(1.0, self._publish_score)

    def _odom_callback(self, msg: Odometry) -> None:
        now_s = _stamp_s(self)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        speed = abs(float(msg.twist.twist.linear.x))
        self.max_speed_mps = max(self.max_speed_mps, speed)

        if self.last_pose is not None:
            self.distance_m += math.hypot(x - self.last_pose[0],
                                          y - self.last_pose[1])
        self.last_pose = (x, y, yaw)
        self.last_time_s = now_s
        self._update_speed_checks(now_s, speed)
        self._check_course_contact(x, y, yaw)
        self._check_finish(x, y)

    def _update_speed_checks(self, now_s: float, speed: float) -> None:
        if self.speed_check_start_s is None:
            # The sim stack can publish odom for many seconds before the
            # mission runner sends the first waypoint. The IGVC 44 ft speed
            # check starts when the robot actually begins the run, not while
            # it is parked during bringup.
            if speed < 0.05 and self.distance_m < 0.05:
                return
            self.speed_check_start_s = now_s
            self.speed_check_start_distance_m = self.distance_m
        if (self.speed_check_end_s is None
                and self.distance_m - self.speed_check_start_distance_m
                >= self.course.speed_check.end_distance_m):
            self.speed_check_end_s = now_s
            elapsed = max(1e-6, now_s - self.speed_check_start_s)
            avg = self.course.speed_check.end_distance_m / elapsed
            if avg < self.course.speed_check.minimum_average_mps:
                self._fail(
                    "first_44ft_speed_below_1mph: %.3f m/s" % avg)
        if speed > self.course.speed_check.maximum_speed_mps:
            self._fail("max_speed_exceeded: %.3f m/s" % speed)
        if speed < 0.02:
            if self.stop_started_s is None:
                self.stop_started_s = now_s
            elif now_s - self.stop_started_s > self.course.speed_check.blocking_stop_s:
                self._fail("blocking_stop_over_60s")
        else:
            self.stop_started_s = None

    def _check_finish(self, x: float, y: float) -> None:
        fx, fy, radius = self.course.finish
        if math.hypot(x - fx, y - fy) <= radius:
            self.finish_reached = True

    def _check_course_contact(self, base_x: float, base_y: float,
                              yaw: float) -> None:
        nav_x = (
            base_x + self.robot.base_link_to_nav_center_m * math.cos(yaw))
        nav_y = (
            base_y + self.robot.base_link_to_nav_center_m * math.sin(yaw))
        hx = self.robot.physical_half_length_m + self.robot.footprint_padding_m
        hy = self.robot.physical_half_width_m + self.robot.footprint_padding_m

        for tape in self.course.tapes:
            if self._segment_hits_body(
                    tape.start, tape.end, tape.width_m * 0.5,
                    nav_x, nav_y, yaw, hx, hy):
                self._fail("tape_crossing:" + tape.name)
        for obstacle in self.course.obstacles:
            if self._circle_hits_body(
                    obstacle.center, obstacle.radius_m, nav_x, nav_y,
                    yaw, hx, hy):
                self._fail("obstacle_contact:" + obstacle.name)
        for pothole in self.course.potholes:
            if self._circle_hits_body(
                    pothole.center, pothole.radius_m, nav_x, nav_y,
                    yaw, hx, hy):
                self._fail("pothole_contact:" + pothole.name)
        for ramp in self.course.ramps:
            if ramp.start_x_m <= nav_x <= ramp.end_x_m:
                if abs(nav_y - ramp.center_y_m) > ramp.width_m * 0.5 + hy:
                    self._fail("ramp_edge_departure:" + ramp.name)

    def _segment_hits_body(self,
                           start: tuple[float, float],
                           end: tuple[float, float],
                           half_width: float,
                           nav_x: float,
                           nav_y: float,
                           yaw: float,
                           hx: float,
                           hy: float) -> bool:
        length = max(0.0, math.hypot(end[0] - start[0], end[1] - start[1]))
        samples = max(1, int(math.ceil(length / self.sample_spacing_m)))
        for idx in range(samples + 1):
            t = idx / float(samples)
            x = start[0] + (end[0] - start[0]) * t
            y = start[1] + (end[1] - start[1]) * t
            local_x, local_y = self._world_to_nav(x, y, nav_x, nav_y, yaw)
            if abs(local_x) <= hx + half_width and abs(local_y) <= hy + half_width:
                return True
        return False

    def _circle_hits_body(self,
                          center: tuple[float, float],
                          radius_m: float,
                          nav_x: float,
                          nav_y: float,
                          yaw: float,
                          hx: float,
                          hy: float) -> bool:
        local_x, local_y = self._world_to_nav(
            center[0], center[1], nav_x, nav_y, yaw)
        dx = max(abs(local_x) - hx, 0.0)
        dy = max(abs(local_y) - hy, 0.0)
        return math.hypot(dx, dy) <= radius_m

    @staticmethod
    def _world_to_nav(x: float, y: float, nav_x: float, nav_y: float,
                      yaw: float) -> tuple[float, float]:
        dx = x - nav_x
        dy = y - nav_y
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def _fail(self, reason: str) -> None:
        if reason not in self.failures:
            self.failures.append(reason)
            self.get_logger().error("IGVC sim failure: %s" % reason)

    def _publish_score(self) -> None:
        score = {
            "course_id": self.course.course_id,
            "failed": bool(self.failures),
            "failures": self.failures,
            "distance_m": round(self.distance_m, 3),
            "max_speed_mps": round(self.max_speed_mps, 3),
            "finish_reached": self.finish_reached,
            "speed_check_complete": self.speed_check_end_s is not None,
        }
        self.score_pub.publish(String(data=json.dumps(score, sort_keys=True)))
        self.fail_pub.publish(Bool(data=bool(self.failures)))


def main(argv: list[str] | None = None) -> int:
    _ = argv
    rclpy.init(args=sys.argv)
    node = IgvcCourseMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
