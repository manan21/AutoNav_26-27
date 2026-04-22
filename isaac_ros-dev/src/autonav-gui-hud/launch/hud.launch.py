from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonav_gui_hud',
            executable='hud_node',
            name='autonav_hud',
            output='screen',
        ),
    ])
