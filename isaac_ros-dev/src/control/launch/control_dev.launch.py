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
                # device_name pin reverted: SDL2 reports the controller
                # with a string that doesn't substring-match the kernel
                # "Xbox Wireless Controller" name, so joy_node logged
                # "Could not get joystick with name ..." and never
                # connected. Falling back to device_id=0 (default —
                # first SDL2-enumerated joystick) until we capture the
                # actual SDL2-reported name on the Jetson; revisit the
                # X/Y-swap problem then.
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

