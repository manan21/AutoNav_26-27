import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch import LaunchDescription
from launch_ros.actions import Node

'''
This launch file is for control testing.
It starts the control node and automatically calls a service that configures the node. 
Please adjust node parameters under control/config/params.yaml

'''

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'node_params.yaml'
    )
    config_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'config_params.yaml'
    ) # bruh
    with open(config_config, 'r') as f:
        all_params = yaml.safe_load(f)
        service_params = all_params.get('configure', {})  
        param_str = str(service_params).replace("'", '"')

    joy = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{
                # Republish /joy at 100 Hz when the stick is held still
                # instead of the joy package default 20 Hz. Combined with
                # control_node consuming /joy directly (not via the 30 ms
                # encoder timer), this drops manual stick → motor latency
                # from 30-50 ms to <10 ms. Autopath unaffected — cmd_vel
                # is on its own subscription.
                'autorepeat_rate': 100.0,
                # Tighter deadzone for more responsive stick feel; the
                # Xbox controller's mechanical center is well inside 0.02.
                'deadzone': 0.02,
                # Bind by NAME, not enumeration order. The default
                # `device_id: 0` picks SDL2's first-found joystick,
                # which on this robot swaps between /dev/input/js0
                # and /dev/input/js1 depending on whether the Xbox
                # controller pairs over Bluetooth before or after
                # the container starts. That order-dependent bind
                # also flips which HID-descriptor mode the kernel
                # exposes (xpad vs hid-microsoft), which is what
                # makes the X/Y button indices swap. Pinning by
                # device_name forces SDL2 to find the controller
                # regardless of jsX number — substring match against
                # the kernel-reported name "Xbox Wireless Controller".
                'device_name': 'Xbox Wireless Controller',
            }],
        )

    control = Node(
            package='control',
            executable='control',
            name='control_node',
            parameters=[config]
        )

    configure = ExecuteProcess(
            cmd=[
                [FindExecutable(name='ros2')],
                ' service call ',
                '/configure_control ',
                'autonav_interfaces/srv/ConfigureControl ',
                f"'{param_str}'"
            ],
            shell=True,
            # delay to ensure the node is running before calling the service
            prefix='sleep 1 && '
        )

    return LaunchDescription([
        joy,
        control,
        configure
    ])

