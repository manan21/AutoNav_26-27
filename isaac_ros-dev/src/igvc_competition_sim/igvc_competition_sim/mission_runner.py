#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

from .course import MissionWaypoint, load_course, local_to_latlon

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from autonav_interfaces.action import NavigateToWaypoint
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_mission_runner must run in a sourced ROS 2 Humble environment"
    ) from exc


class MissionRunner(Node):
    def __init__(self, course_config: str, timeout_sec: float) -> None:
        super().__init__("igvc_mission_runner")
        self.course = load_course(course_config or None)
        self.timeout_sec = timeout_sec
        self.client = ActionClient(
            self, NavigateToWaypoint, "/navigate_to_waypoint")

    def run(self) -> bool:
        self.get_logger().info(
            "waiting for /navigate_to_waypoint action server")
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/navigate_to_waypoint unavailable")
            return False
        for waypoint in self.course.mission_waypoints:
            if not self._send_and_wait(waypoint):
                self.get_logger().error(
                    "mission aborted at waypoint %s" % waypoint.label)
                return False
        self.get_logger().info("mission complete")
        return True

    def _send_and_wait(self, waypoint: MissionWaypoint) -> bool:
        goal = NavigateToWaypoint.Goal()
        target = PoseStamped()
        target.pose.orientation.w = 1.0
        goal.success_radius_m = waypoint.radius_m
        if waypoint.kind == "gps":
            lat, lon = local_to_latlon(
                waypoint.x_m,
                waypoint.y_m,
                self.course.datum_latitude_deg,
                self.course.datum_longitude_deg,
            )
            goal.goal_type = NavigateToWaypoint.Goal.GOAL_TYPE_GPS
            target.header.frame_id = "wgs84"
            target.pose.position.x = lon
            target.pose.position.y = lat
            target.pose.position.z = self.course.datum_altitude_m
            detail = "lat=%.8f lon=%.8f" % (lat, lon)
        else:
            goal.goal_type = NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL
            target.header.frame_id = "map"
            target.pose.position.x = waypoint.x_m
            target.pose.position.y = waypoint.y_m
            detail = "x=%.2f y=%.2f" % (waypoint.x_m, waypoint.y_m)
        goal.target = target
        self.get_logger().info(
            "[%s] sending %s waypoint %s radius=%.2f"
            % (waypoint.label, waypoint.kind, detail, waypoint.radius_m))

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self.timeout_sec)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("[%s] goal rejected" % waypoint.label)
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.timeout_sec)
        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error("[%s] result timed out" % waypoint.label)
            return False
        result = result_response.result
        self.get_logger().info(
            "[%s] status=%d succeeded=%s final_distance=%.3f reason=%s"
            % (
                waypoint.label,
                result.terminal_status,
                result.succeeded,
                result.final_distance_m,
                result.failure_reason,
            )
        )
        return bool(result.succeeded)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--course-config", default="")
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    raw_args = sys.argv[1:] if argv is None else argv
    args, ros_args = parser.parse_known_args(raw_args)

    rclpy.init(args=([sys.argv[0]] + ros_args) if argv is None else ros_args)
    node = MissionRunner(args.course_config, args.timeout_sec)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
