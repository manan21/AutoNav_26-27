# Brings up detectors in this package:
#   - line_detector  (CUDA, ZED RGB+depth → painted-line points)
#   - grade_detector (Eigen, SICK PointCloud2 → PCA-filtered obstacle cloud)
#   - lidar_line_detector (SICK PointCloud2 RSSI → painted-line points)
#
# Single Ctrl-C brings them both down. Parameters are loaded from the
# package's own config/ directory (NOT nav2_paramsv2.yaml — algorithm
# tuning lives with the package).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonav_detection')

    line_yaml_arg = DeclareLaunchArgument(
        'line_detector_params',
        default_value=os.path.join(pkg_share, 'config', 'line_detector.yaml'),
        description='YAML for the camera-based line detector.',
    )
    grade_yaml_arg = DeclareLaunchArgument(
        'grade_detector_params',
        default_value=os.path.join(pkg_share, 'config', 'grade_detector.yaml'),
        description='YAML for the LiDAR PCA grade detector.',
    )
    lidar_line_yaml_arg = DeclareLaunchArgument(
        'lidar_line_detector_params',
        default_value=os.path.join(pkg_share, 'config', 'lidar_line_detector.yaml'),
        description='YAML for the LiDAR RSSI line detector.',
    )
    enable_line = DeclareLaunchArgument(
        'enable_line', default_value='true',
        description='Set false to skip the CUDA line detector (e.g. dev box without ZED).',
    )
    enable_grade = DeclareLaunchArgument(
        'enable_grade', default_value='true',
        description='Set false to skip the PCA grade detector.',
    )
    enable_lidar_line = DeclareLaunchArgument(
        'enable_lidar_line', default_value='true',
        description='Set false to skip the LiDAR RSSI line detector.',
    )

    line_detector = Node(
        package='autonav_detection',
        executable='line_detector',
        # Keep the historical node name so /line_points and downstream
        # consumers (line_layer plugin, automated_testing) keep working.
        name='line_detection_node',
        output='screen',
        parameters=[LaunchConfiguration('line_detector_params')],
        condition=IfCondition(LaunchConfiguration('enable_line')),
    )
    grade_detector = Node(
        package='autonav_detection',
        executable='grade_detector',
        name='grade_detector',
        output='screen',
        parameters=[LaunchConfiguration('grade_detector_params')],
        condition=IfCondition(LaunchConfiguration('enable_grade')),
    )
    lidar_line_detector = Node(
        package='autonav_detection',
        executable='lidar_line_detector',
        name='lidar_line_detector',
        output='screen',
        parameters=[LaunchConfiguration('lidar_line_detector_params')],
        condition=IfCondition(LaunchConfiguration('enable_lidar_line')),
    )

    return LaunchDescription([
        line_yaml_arg,
        grade_yaml_arg,
        lidar_line_yaml_arg,
        enable_line,
        enable_grade,
        enable_lidar_line,
        line_detector,
        grade_detector,
        lidar_line_detector,
    ])
