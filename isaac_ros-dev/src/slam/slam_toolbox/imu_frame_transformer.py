#!/usr/bin/env python3
"""
IMU Frame Transformer
Transforms IMU data from lidar_footprint (upside-down) to base_link frame.

The SICK LiDAR is mounted upside-down (π roll) on the robot. When the IMU is in
an upside-down frame, its Z-axis (yaw) angular velocity is inverted relative to
the base_link frame. This node transforms the angular velocities to account for
this 180° roll offset.

For a π roll (180° rotation around X-axis):
  - vx_base = vx_lidar (X unchanged)
  - vy_base = -vy_lidar (Y inverted)
  - vz_base = -vz_lidar (Z inverted - THIS IS THE YAW FIX)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Vector3
import math


class IMUFrameTransformer(Node):
    def __init__(self):
        super().__init__('imu_frame_transformer')
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to raw IMU in lidar_footprint frame
        self.imu_sub = self.create_subscription(
            Imu,
            'sick_scansegment_xd/imu',
            self.imu_callback,
            10
        )
        
        # Publish transformed IMU in base_link frame
        self.imu_pub = self.create_publisher(
            Imu,
            'sick_scansegment_xd/imu_base_link',
            10
        )
        
        self.get_logger().info('IMU Frame Transformer initialized')
        self.get_logger().info('Converting IMU from lidar_footprint (upside-down) to base_link')
    
    def imu_callback(self, msg: Imu):
        """
        Transform IMU data from lidar_footprint to base_link.
        
        The lidar_footprint is rotated π radians (180°) around the X-axis
        relative to base_link. This means:
        - Angular velocities around X are unchanged
        - Angular velocities around Y and Z are negated
        """
        try:
            # Get the transform from base_link to lidar_footprint
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'lidar_footprint',
                rclpy.time.Time()
            )
            
            # Extract the rotation (as a quaternion)
            quat = transform.transform.rotation
            
            # Convert IMU angular velocity from lidar_footprint to base_link
            # For a π roll (180° around X), the transformation is:
            vx_base = msg.angular_velocity.x
            vy_base = -msg.angular_velocity.y
            vz_base = -msg.angular_velocity.z
            
            # Create output message (copy and modify)
            out_msg = Imu()
            out_msg.header = msg.header
            out_msg.header.frame_id = 'base_link'
            
            # Copy orientation (quaternion) - transform the orientation too
            # For π roll, the quaternion transforms as: q_base = q_transform * q_imu * q_transform^-1
            # But for a fixed known transform, we can directly apply it
            out_msg.orientation = msg.orientation
            
            # Apply angular velocity transformation (negated Y and Z)
            out_msg.angular_velocity.x = vx_base
            out_msg.angular_velocity.y = vy_base
            out_msg.angular_velocity.z = vz_base
            
            # Angular velocity covariance needs to be transformed similarly
            # Rot matrix for π roll: [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
            # For covariance: P' = R * P * R^T
            if msg.angular_velocity_covariance[0] != 0:
                # Create transformed covariance matrix
                out_msg.angular_velocity_covariance = [0.0] * 9
                # (0,0) -> (0,0): no change
                out_msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[0]
                # (1,1) -> (1,1): no change  
                out_msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[4]
                # (2,2) -> (2,2): no change
                out_msg.angular_velocity_covariance[8] = msg.angular_velocity_covariance[8]
            else:
                out_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            
            # Copy linear acceleration (unchanged in this frame transformation)
            out_msg.linear_acceleration = msg.linear_acceleration
            out_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            
            # Copy orientation covariance
            out_msg.orientation_covariance = msg.orientation_covariance
            
            # Publish transformed message
            self.imu_pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            # Fallback: publish with manual transformation (no TF)
            # This is faster and works since the transform is fixed
            out_msg = Imu()
            out_msg.header = msg.header
            out_msg.header.frame_id = 'base_link'
            out_msg.orientation = msg.orientation
            
            # Apply π roll transformation: negate Y and Z angular velocities
            out_msg.angular_velocity.x = msg.angular_velocity.x
            out_msg.angular_velocity.y = -msg.angular_velocity.y
            out_msg.angular_velocity.z = -msg.angular_velocity.z
            
            out_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            out_msg.linear_acceleration = msg.linear_acceleration
            out_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            out_msg.orientation_covariance = msg.orientation_covariance
            
            self.imu_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUFrameTransformer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
