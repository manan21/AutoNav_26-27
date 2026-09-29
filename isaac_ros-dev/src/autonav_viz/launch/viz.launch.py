# Brings up rviz2 with rviz/cv_pipeline.rviz (lanes, lidar obstacles,
# costmaps and path). See README.md for the topics it displays.
#
# Examples:
#   ros2 launch autonav_viz viz.launch.py
#   ros2 launch autonav_viz viz.launch.py use_sim_time:=true        # rosbag replay

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        rviz,
    ])
