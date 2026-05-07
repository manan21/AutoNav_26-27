import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


'''
Core launch. Handles robot_description and TF from the robot model.
'''


def _load_robot_description(model_path):
    if model_path.endswith('.xacro'):
        import xacro
        return xacro.process_file(model_path).toxml()

    with open(model_path, 'r', encoding='utf-8') as model_file:
        return model_file.read()


def _launch_setup(context, *args, **kwargs):
    model_file = LaunchConfiguration('model').perform(context)
    model_path = os.path.join(
        get_package_share_directory('bringup'),
        'description',
        model_file,
    )

    params = {'robot_description': _load_robot_description(model_path)}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[model_path]
    )

    return [rsp, jsp]


def generate_launch_description():
    model = DeclareLaunchArgument(
        'model',
        default_value='shogi.urdf',
        description='Robot model file from the bringup description directory'
    )

    return LaunchDescription([
        model,
        OpaqueFunction(function=_launch_setup),
    ])
