"""
Map Padder Node

Subscribes to the SLAM /map topic and republishes a padded version on
/map_padded.  The padded map:

  - Is always at least min_width_m x min_height_m (default 100 x 100 m)
  - Always contains the robot's current position with a buffer (default 20 m)
  - Always contains the latest navigation goal with a buffer (default 20 m)

The original SLAM data is preserved inside the larger grid; surrounding
cells are filled with -1 (unknown).

This ensures the global costmap is large enough for distant GPS waypoints
and that the robot is never outside its own costmap.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException


class MapPadder(Node):

    def __init__(self):
        super().__init__('map_padder')

        self.declare_parameter('min_width_m', 100.0)
        self.declare_parameter('min_height_m', 100.0)
        self.declare_parameter('robot_buffer_m', 20.0)
        self.declare_parameter('goal_buffer_m', 20.0)
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_padded')
        self.declare_parameter('goal_topic', '/goal_pose')

        self._min_width_m = self.get_parameter('min_width_m').value
        self._min_height_m = self.get_parameter('min_height_m').value
        self._robot_buffer_m = self.get_parameter('robot_buffer_m').value
        self._goal_buffer_m = self.get_parameter('goal_buffer_m').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        goal_topic = self.get_parameter('goal_topic').value

        # TF for looking up robot position in the map frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Latest goal and map (so we can re-pad when a new goal arrives)
        self._latest_goal = None   # (x, y) in map frame
        self._latest_map = None    # OccupancyGrid

        # Match SLAM toolbox QoS (transient-local, reliable)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._pub = self.create_publisher(OccupancyGrid, output_topic, map_qos)
        self._sub = self.create_subscription(
            OccupancyGrid, input_topic, self._on_map, map_qos)

        # Subscribe to navigation goals so we can expand the map to include them
        self._goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal, 10)

        self.get_logger().info(
            f'Map padder ready  min_size={self._min_width_m}x{self._min_height_m}m  '
            f'robot_buffer={self._robot_buffer_m}m  goal_buffer={self._goal_buffer_m}m  '
            f'{input_topic} -> {output_topic}')

    def _on_goal(self, msg: PoseStamped):
        """Store latest goal and re-pad the map to include it."""
        self._latest_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'New goal at ({self._latest_goal[0]:.1f}, {self._latest_goal[1]:.1f})')
        # Re-pad with the stored map so the costmap expands immediately
        if self._latest_map is not None:
            self._pad_and_publish(self._latest_map)

    def _get_robot_position(self):
        """Return (x, y) of the robot in the map frame, or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg
        self._pad_and_publish(msg)

    def _pad_and_publish(self, msg: OccupancyGrid):
        res = msg.info.resolution
        if res <= 0.0:
            self.get_logger().warn('Received map with zero resolution, skipping')
            return

        orig_w = msg.info.width
        orig_h = msg.info.height
        orig_ox = msg.info.origin.position.x
        orig_oy = msg.info.origin.position.y

        # Bounding box of the original SLAM map in world coords
        slam_min_x = orig_ox
        slam_min_y = orig_oy
        slam_max_x = orig_ox + orig_w * res
        slam_max_y = orig_oy + orig_h * res

        # Start with the SLAM bounds
        need_min_x = slam_min_x
        need_min_y = slam_min_y
        need_max_x = slam_max_x
        need_max_y = slam_max_y

        # Expand to minimum configured size (centred on SLAM map)
        slam_cx = (slam_min_x + slam_max_x) / 2.0
        slam_cy = (slam_min_y + slam_max_y) / 2.0
        need_min_x = min(need_min_x, slam_cx - self._min_width_m / 2.0)
        need_max_x = max(need_max_x, slam_cx + self._min_width_m / 2.0)
        need_min_y = min(need_min_y, slam_cy - self._min_height_m / 2.0)
        need_max_y = max(need_max_y, slam_cy + self._min_height_m / 2.0)

        # Expand to include robot position + buffer
        robot_pos = self._get_robot_position()
        if robot_pos is not None:
            rx, ry = robot_pos
            buf = self._robot_buffer_m
            need_min_x = min(need_min_x, rx - buf)
            need_max_x = max(need_max_x, rx + buf)
            need_min_y = min(need_min_y, ry - buf)
            need_max_y = max(need_max_y, ry + buf)

        # Expand to include navigation goal + buffer
        if self._latest_goal is not None:
            gx, gy = self._latest_goal
            buf = self._goal_buffer_m
            need_min_x = min(need_min_x, gx - buf)
            need_max_x = max(need_max_x, gx + buf)
            need_min_y = min(need_min_y, gy - buf)
            need_max_y = max(need_max_y, gy + buf)

        # Convert world bounds to cell dimensions (snap to resolution grid)
        pad_ox = math.floor(need_min_x / res) * res
        pad_oy = math.floor(need_min_y / res) * res
        pad_w = math.ceil((need_max_x - pad_ox) / res)
        pad_h = math.ceil((need_max_y - pad_oy) / res)

        # Where the original SLAM data sits inside the padded grid
        off_x = round((orig_ox - pad_ox) / res)
        off_y = round((orig_oy - pad_oy) / res)

        if pad_w == orig_w and pad_h == orig_h and off_x == 0 and off_y == 0:
            # No padding needed
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(msg)
            return

        # Build padded grid filled with -1 (unknown)
        padded = np.full((pad_h, pad_w), -1, dtype=np.int8)

        # Copy original SLAM data in
        orig = np.array(msg.data, dtype=np.int8).reshape((orig_h, orig_w))
        padded[off_y:off_y + orig_h, off_x:off_x + orig_w] = orig

        out = OccupancyGrid()
        out.header = msg.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = res
        out.info.width = pad_w
        out.info.height = pad_h
        out.info.origin.position.x = pad_ox
        out.info.origin.position.y = pad_oy
        out.info.origin.position.z = msg.info.origin.position.z
        out.info.origin.orientation = msg.info.origin.orientation
        out.data = padded.flatten().tolist()

        self._pub.publish(out)
        self.get_logger().info(
            f'Padded map {orig_w}x{orig_h} -> {pad_w}x{pad_h} cells '
            f'({pad_w * res:.0f}x{pad_h * res:.0f} m)'
            f'{" [robot included]" if robot_pos else " [no robot TF yet]"}'
            f'{" [goal included]" if self._latest_goal else ""}')


def main(args=None):
    rclpy.init(args=args)
    node = MapPadder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
