#!/usr/bin/env python3
from __future__ import annotations

import math
import sys

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_camera_bridge must run in a sourced ROS 2 Humble environment"
    ) from exc

# RCLError was added to rclpy after Humble; fall back when absent (matches the
# pattern already in dynamics_replay.py). auto_camera env-compat fix.
try:
    from rclpy.exceptions import RCLError
except ImportError:
    RCLError = Exception


def _q_from_rpy(roll: float, pitch: float, yaw: float
                ) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class IgvcCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("igvc_camera_bridge")

        self.declare_parameter("input_image_topic", "/igvc_sim/zed/image")
        self.declare_parameter("input_depth_topic", "/igvc_sim/zed/depth_image")
        self.declare_parameter(
            "input_camera_info_topic", "/igvc_sim/zed/camera_info")
        self.declare_parameter(
            "output_image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter(
            "output_depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter(
            "output_camera_info_topic",
            "/zed/zed_node/rgb/color/rect/camera_info")
        self.declare_parameter(
            "output_depth_info_topic", "/zed/zed_node/depth/depth_info")
        self.declare_parameter("camera_link_frame_id", "zed_camera_link")
        self.declare_parameter("camera_frame_id", "zed2i_left_camera_frame")
        self.declare_parameter(
            "optical_frame_id", "zed2i_left_camera_frame_optical")
        self.declare_parameter("fallback_width", 960)
        self.declare_parameter("fallback_height", 540)
        self.declare_parameter("fallback_horizontal_fov_rad", 1.918862)

        self.optical_frame_id = str(
            self.get_parameter("optical_frame_id").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.camera_link_frame_id = str(
            self.get_parameter("camera_link_frame_id").value)
        self.fallback_width = int(self.get_parameter("fallback_width").value)
        self.fallback_height = int(self.get_parameter("fallback_height").value)
        self.fallback_horizontal_fov_rad = float(
            self.get_parameter("fallback_horizontal_fov_rad").value)

        image_qos = QoSProfile(depth=10)
        info_qos = QoSProfile(depth=1)

        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_image_topic").value),
            image_qos,
        )
        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_depth_topic").value),
            image_qos,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("output_camera_info_topic").value),
            info_qos,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("output_depth_info_topic").value),
            info_qos,
        )

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("input_image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image,
            str(self.get_parameter("input_depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("input_camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_static_camera_transforms()

        self.get_logger().info(
            "Relaying Gazebo RGB-D camera into ZED topics with frame %s"
            % self.optical_frame_id
        )

    def _stamp_if_zero(self, msg) -> None:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

    def _image_callback(self, msg: Image) -> None:
        self._stamp_if_zero(msg)
        msg.header.frame_id = self.optical_frame_id
        self.image_pub.publish(msg)

    def _depth_callback(self, msg: Image) -> None:
        self._stamp_if_zero(msg)
        msg.header.frame_id = self.optical_frame_id
        self.depth_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._stamp_if_zero(msg)
        msg.header.frame_id = self.optical_frame_id
        self._fill_camera_info_if_empty(msg)
        self.camera_info_pub.publish(msg)
        self.depth_info_pub.publish(msg)

    def _fill_camera_info_if_empty(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            return
        width = int(msg.width) if msg.width else self.fallback_width
        height = int(msg.height) if msg.height else self.fallback_height
        msg.width = width
        msg.height = height
        fx = (0.5 * width) / math.tan(0.5 * self.fallback_horizontal_fov_rad)
        fy = fx
        cx = 0.5 * (width - 1)
        cy = 0.5 * (height - 1)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def _publish_static_camera_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        to_camera_frame = TransformStamped()
        to_camera_frame.header.stamp = stamp
        to_camera_frame.header.frame_id = self.camera_link_frame_id
        to_camera_frame.child_frame_id = self.camera_frame_id
        to_camera_frame.transform.rotation.w = 1.0

        to_optical = TransformStamped()
        to_optical.header.stamp = stamp
        to_optical.header.frame_id = self.camera_frame_id
        to_optical.child_frame_id = self.optical_frame_id
        qx, qy, qz, qw = _q_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        to_optical.transform.rotation.x = qx
        to_optical.transform.rotation.y = qy
        to_optical.transform.rotation.z = qz
        to_optical.transform.rotation.w = qw

        self.tf_static.sendTransform([to_camera_frame, to_optical])


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=sys.argv if argv is None else argv)
    node = IgvcCameraBridge()
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
