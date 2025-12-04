import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

'''
Launch script for [TEST ID: t000] DAQ Mode automated test.

Launching this script will:
1. Start the data_publisher node to collect broad DAQ data
2. Launch standard bringup and control (for operator control)
3. Execute t000_automator.py which starts/stops data collection on A button

Notes:
- No camera image topics are used for DAQ here.
- This is a framework to expose more numeric topics into the CSV.
'''

def generate_launch_description():

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model'
    )

    # Package directories
    autonav_testing_share = get_package_share_directory('autonav_automated_testing')
    zed_pkg = os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
    control_share = FindPackageShare('control')
    gps_share = FindPackageShare('gps_handler')
    odom_share = FindPackageShare('odom_handler')

    # Path to test data configuration file
    test_data_config = os.path.join(
        autonav_testing_share,
        'config',
        'testing_data_collection_setter.yaml'
    )

    # Load topics to monitor for this specific test
    with open(test_data_config, 'r') as f:
        all_params = yaml.safe_load(f)
        data_publisher_params = all_params.get('data_publisher', {}).get('ros__parameters', {})
        topics_to_monitor = data_publisher_params.get('t000', [])

    # Data Publisher Node
    data_publisher_node = Node(
        package='autonav_automated_testing',
        executable='data_publisher',
        name='data_publisher_node',
        output='screen',
        parameters=[{
            'topics_to_monitor': list(topics_to_monitor),
            'test_id': 't000',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # [NODES]
    # GPS Handler Node - publishes GPS fix data to /gps/fix
    gps_handler_node = Node(
        package='gps_handler',
        executable='gps_publisher',
        name='gps_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gps_port': '/dev/ttyUSB0'
        }]
    )

    # Odometry Handler Node - publishes wheel encoder data to /encoders and /odom
    odom_handler_node = Node(
        package='odom_handler',
        executable='wheel_odometry_publisher',
        name='odom_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Include ZED camera launch file for camera and IMU data
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_pkg),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include control launch file for motor control and /cmd_vel processing
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([control_share, 'launch', 'control_dev.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Execute the test automator script
    automater_script_path = os.path.join(
        autonav_testing_share,
        'src',
        'automators',
        't000_automator.py'
    )
    test_automater = ExecuteProcess(
        cmd=[
            'python3',
            automater_script_path,
            '--ros-args',
            '-p',
            PythonExpression([
                "'use_sim_time:=' + '", LaunchConfiguration('use_sim_time'), "'"
            ])
        ],
        output='screen',
        name='t000_automater'
    )

    return LaunchDescription([
        use_sim_time,
        camera_model,
        # [NODES]
        gps_handler_node,
        odom_handler_node,
        # [LAUNCH FILES]
        zed_launch,
        control_launch,
        # Data collection
        data_publisher_node,
        # Automator
        test_automater
    ])