# Brings up the CV-pipeline RViz suite:
#   - rviz2 with rviz/cv_pipeline.rviz (lanes, obstacles, signals, BEV)
#   - perception_viz_stub (optional) — placeholder data for perception
#     topics that no real detector publishes yet. See README.md for the
#     topic contract.
#
# Examples:
#   ros2 launch autonav_viz viz.launch.py
#   ros2 launch autonav_viz viz.launch.py use_stub:=true
#   ros2 launch autonav_viz viz.launch.py use_stub:=true stub_classes:=barrel,traffic_light
#   ros2 launch autonav_viz viz.launch.py use_sim_time:=true        # rosbag replay

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('autonav_viz')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'cv_pipeline.rviz'),
        description='RViz config to open.',
    )
    use_stub_arg = DeclareLaunchArgument(
        'use_stub', default_value='false',
        description='Start perception_viz_stub to fill not-yet-implemented perception topics.',
    )
    stub_classes_arg = DeclareLaunchArgument(
        'stub_classes', default_value='all',
        description='Comma list of stub object classes (barrel,pothole,pedestrian,'
                    'stop_sign,road_sign,traffic_light) or "all".',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (sim or rosbag replay with --clock).',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_cv_pipeline',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    stub = Node(
        package='autonav_viz',
        executable='perception_viz_stub',
        name='perception_viz_stub',
        output='screen',
        parameters=[{
            'classes': LaunchConfiguration('stub_classes'),
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('use_stub')),
    )

    return LaunchDescription([
        rviz_config_arg,
        use_stub_arg,
        stub_classes_arg,
        use_sim_time_arg,
        rviz,
        stub,
    ])
