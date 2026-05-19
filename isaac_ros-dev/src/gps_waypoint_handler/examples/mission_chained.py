#!/usr/bin/env python3
# isaac_ros-dev/src/gps_waypoint_handler/examples/mission_chained.py
#
# Example mission script: chains three heterogeneous waypoints (GPS,
# local, GPS) through the single /navigate_to_waypoint action client.
# Demonstrates the unified-action pattern from §13 #11 of the plan —
# heterogeneous waypoint chains do NOT need two separate actions.
#
# Run with:
#   python3 mission_chained.py
#
# Requires: gps_handler_node running, ros2 environment sourced,
# autonav_interfaces built.

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from autonav_interfaces.action import NavigateToWaypoint
from geometry_msgs.msg import PoseStamped


# Surveyed test waypoint from plan_manifest.md §6.3
GPS_WAYPOINT_A = (37.23027, -80.42504)
GPS_WAYPOINT_B = (37.23030, -80.42508)
LOCAL_WAYPOINT = (2.0, 0.0)  # in map frame


def make_gps_goal(lat: float, lon: float, radius_m: float = 1.0):
    """Build a NavigateToWaypoint.Goal for a GPS target (lat/lon)."""
    goal = NavigateToWaypoint.Goal()
    goal.goal_type = NavigateToWaypoint.Goal.GOAL_TYPE_GPS
    target = PoseStamped()
    target.header.frame_id = 'wgs84'
    # GPS convention: x = longitude, y = latitude
    target.pose.position.x = lon
    target.pose.position.y = lat
    target.pose.position.z = 0.0
    target.pose.orientation.w = 1.0  # identity = auto yaw
    goal.target = target
    goal.success_radius_m = radius_m
    return goal


def make_local_goal(x: float, y: float, radius_m: float = 1.0,
                    frame_id: str = 'map'):
    """Build a NavigateToWaypoint.Goal for a local (map/odom) target."""
    goal = NavigateToWaypoint.Goal()
    goal.goal_type = NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL
    target = PoseStamped()
    target.header.frame_id = frame_id
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = 0.0
    target.pose.orientation.w = 1.0
    goal.target = target
    goal.success_radius_m = radius_m
    return goal


def send_and_wait(node: Node, client: ActionClient, goal, label: str) -> bool:
    """Send a single goal and block until terminal. Returns True on success."""
    node.get_logger().info(f'[{label}] waiting for action server...')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f'[{label}] action server unavailable')
        return False

    node.get_logger().info(f'[{label}] sending goal')
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error(f'[{label}] goal rejected')
        return False

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    node.get_logger().info(
        f'[{label}] terminal_status={result.terminal_status} '
        f'succeeded={result.succeeded} reason="{result.failure_reason}"')
    return bool(result.succeeded)


def main(argv=None):
    rclpy.init(args=argv)
    node = rclpy.create_node('mission_chained_example')
    client = ActionClient(node, NavigateToWaypoint, '/navigate_to_waypoint')

    legs = [
        ('gps_A', make_gps_goal(*GPS_WAYPOINT_A)),
        ('local', make_local_goal(*LOCAL_WAYPOINT)),
        ('gps_B', make_gps_goal(*GPS_WAYPOINT_B)),
    ]

    overall_ok = True
    for label, goal in legs:
        ok = send_and_wait(node, client, goal, label)
        if not ok:
            node.get_logger().error(f'mission aborted at leg "{label}"')
            overall_ok = False
            break

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if overall_ok else 1)


if __name__ == '__main__':
    main()
