# IMU Yaw Fix: SICK LiDAR Upside-Down Mounting Issue

## Problem Diagnosis

The SICK LiDAR IMU was sending inverted yaw information because:

1. **LiDAR Mounting**: The LiDAR is mounted upside-down on the robot chassis with a π radian (180°) roll rotation relative to `base_link`. This is defined in the URDF:
   ```xml
   <joint name="lidar_joint" type="fixed">
     <parent link="base_link"/>
     <child link="lidar_footprint"/>
     <origin xyz="0.44 0.0 0.15" rpy="3.1415 0 0"/>  <!-- π roll = upside-down -->
   </joint>
   ```

2. **IMU Frame Issue**: The IMU publishes data in the `lidar_footprint` frame, which is rotated 180° around the X-axis relative to `base_link`.

3. **Axis Inversion**: When an IMU is rotated 180° around the X-axis:
   - X angular velocity (roll) → unchanged
   - Y angular velocity (pitch) → inverted
   - **Z angular velocity (yaw) → inverted** ← **THIS WAS THE PROBLEM**

4. **No Compensation in EKF**: The original `ekf_local.yaml` was directly subscribing to `/sick_scansegment_xd/imu` without any frame transformation, causing the inverted yaw data to be fused directly into the odometry estimate.

## Solution

Created an **IMU Frame Transformer** node that:

1. **Subscribes** to the raw IMU data in `lidar_footprint` frame
2. **Transforms** angular velocities to correct for the 180° roll offset:
   - `vz_base = -vz_lidar` (inverts yaw rate)
   - `vy_base = -vy_lidar` (inverts pitch rate for consistency)
3. **Publishes** the corrected IMU data on `/sick_scansegment_xd/imu_base_link` in the `base_link` frame

### Files Modified

1. **Created**: `isaac_ros-dev/src/slam/slam_toolbox/imu_frame_transformer.py`
   - New Python node that performs the frame transformation
   - Handles TF lookups for robustness, with fallback to fixed transform
   - Properly transforms both angular velocity and covariance matrices

2. **Updated**: `isaac_ros-dev/src/slam/config/ekf_local.yaml`
   - Changed EKF IMU input from `sick_scansegment_xd/imu` to `sick_scansegment_xd/imu_base_link`
   - Added comment explaining the transformation

3. **Updated**: `isaac_ros-dev/src/slam/launch/slam.launch.py`
   - Added `imu_frame_transformer` node to launch sequence
   - Ensures it starts before EKF consumes the transformed IMU data

4. **Updated**: `isaac_ros-dev/src/slam/CMakeLists.txt`
   - Added `ament_cmake_python` dependency
   - Configured installation of the Python executable

5. **Updated**: `isaac_ros-dev/src/slam/package.xml`
   - Added `ament_cmake_python`, `rclpy`, `sensor_msgs`, `tf2_ros`, `geometry_msgs` dependencies

## Technical Details

### The Transformation
For a 180° rotation around the X-axis (π roll), the rotation matrix is:
```
R = [ 1    0    0  ]
    [ 0   -1    0  ]
    [ 0    0   -1  ]
```

This means:
- Angular velocity components are transformed as: **ω' = R × ω**
- The Z-component is negated: **ωz_base = -ωz_lidar** (fixes yaw)

### Data Flow
```
SICK IMU → /sick_scansegment_xd/imu (lidar_footprint frame, upside-down)
           ↓
     IMU Frame Transformer Node
           ↓
         /sick_scansegment_xd/imu_base_link (base_link frame, corrected)
           ↓
     Local EKF Node
           ↓
         Corrected odometry estimate
```

## Testing the Fix

After rebuilding and restarting the SLAM system:

1. The yaw rate should now be correctly oriented
2. The robot's rotation should be properly tracked by the EKF
3. SLAM and Nav2 should produce correct heading estimates

Monitor the yaw angular velocity values:
```bash
ros2 topic echo /sick_scansegment_xd/imu_base_link --field angular_velocity
```

The Z component should now match the actual yaw rotation of the robot (instead of being inverted).

## Future Considerations

If the LiDAR is ever remounted in a different orientation:
1. Update the URDF joint transform (`rpy` values)
2. Modify the IMU frame transformer to use the correct axis inversion
3. Or use a dynamic TF-based approach (the current code tries TF lookup first as a fallback)
