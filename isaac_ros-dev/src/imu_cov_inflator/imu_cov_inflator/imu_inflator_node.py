"""Republish an IMU topic with inflated covariance + rotated into base_link.

The SICK multiScan IMU on this robot is mounted **upside-down on an
arm**, with frame_id "imu_link" — but no URDF link / static TF exists
for imu_link, so robot_localization can't rotate the readings into
base_link via TF. This node does that rotation in software and emits
the result with ``frame_id: base_link``, so ekf_local consumes the
rotated values directly without needing the missing TF chain.

Also inflates the bare ``angular_velocity_covariance`` / ``linear_
acceleration_covariance`` (which the SICK driver emits as all zeros)
to realistic MEMS-grade variances so robot_localization doesn't
over-fuse the IMU.

The arm-offset (translation from base_link to the IMU) does NOT affect
angular velocity for a rigid body, so the inflator handles rotation
only. We're not fusing linear acceleration, so the centripetal
lever-arm effect on the accelerometer is also not relevant here.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuInflator(Node):
    def __init__(self):
        super().__init__('imu_cov_inflator')

        self.declare_parameter('input_topic', '/sick_scansegment_xd/imu')
        self.declare_parameter('output_topic',
                               '/sick_scansegment_xd/imu_inflated')
        self.declare_parameter('output_frame_id', 'base_link')

        # Rotation from the IMU's body frame to base_link, in radians.
        # Default roll=π matches an IMU mounted upside-down (z and y
        # negated). For other mounts: set the RPY that takes a vector
        # in the IMU body frame to base_link.
        self.declare_parameter('roll', math.pi)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        # σ ≈ 5.7°/s for a typical MEMS gyro.
        self.declare_parameter('angular_velocity_variance', 0.01)
        # σ ≈ 0.3 m/s² for typical MEMS accel.
        self.declare_parameter('linear_acceleration_variance', 0.09)
        # Orientation is rarely trustworthy on a bare gyro.
        self.declare_parameter('orientation_variance', 1.0)

        in_topic = self.get_parameter('input_topic').value
        self._out_topic = self.get_parameter('output_topic').value
        self._out_frame = self.get_parameter('output_frame_id').value

        roll = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw = float(self.get_parameter('yaw').value)
        self._R = _rpy_matrix(roll, pitch, yaw)

        self._w_var = float(
            self.get_parameter('angular_velocity_variance').value)
        self._a_var = float(
            self.get_parameter('linear_acceleration_variance').value)
        self._o_var = float(
            self.get_parameter('orientation_variance').value)

        self._pub = self.create_publisher(
            Imu, self._out_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Imu, in_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'inflator: {in_topic} -> {self._out_topic} '
            f'(frame={self._out_frame}, rpy=({roll:.2f},{pitch:.2f},'
            f'{yaw:.2f}), gyro σ²={self._w_var})')

    def _rotate(self, v):
        R = self._R
        return (
            R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
            R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
            R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
        )

    def _cb(self, msg: Imu):
        wx, wy, wz = self._rotate((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz

        ax, ay, az = self._rotate((
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ))
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.header.frame_id = self._out_frame

        msg.orientation_covariance = [
            self._o_var, 0.0, 0.0,
            0.0, self._o_var, 0.0,
            0.0, 0.0, self._o_var,
        ]
        msg.angular_velocity_covariance = [
            self._w_var, 0.0, 0.0,
            0.0, self._w_var, 0.0,
            0.0, 0.0, self._w_var,
        ]
        msg.linear_acceleration_covariance = [
            self._a_var, 0.0, 0.0,
            0.0, self._a_var, 0.0,
            0.0, 0.0, self._a_var,
        ]
        self._pub.publish(msg)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) Ry(pitch) Rx(roll)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ]


def main(args=None):
    rclpy.init(args=args)
    node = ImuInflator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
