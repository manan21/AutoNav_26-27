import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

'''
Launch script for [TEST ID: t000] DAQ Mode automated test.

IMPORTANT: This launch file is designed to run ALONGSIDE the DEMO_DAY bringup.
Run the 7 DEMO_DAY steps first (pre_slam, zed, lidar, slam, lines, nav2, rviz),
then launch this in an 8th terminal to add DAQ data collection.

Default mode (enable_legacy_capture:=false): the automator only
fires test_actions on A-press; data acquisition is entirely separate.
ROS bag recording is HUD-driven now — the operator presses R in the
HUD to start/stop a `ros2 bag record` subprocess that captures the
canonical topic set (HudWindow._BAG_PLAYBACK_TOPICS). No CSV, no
per-sensor MP4 from this path.

Legacy mode (enable_legacy_capture:=true): restores the older per-
sensor CSV + camera/LiDAR MP4 pipeline (data_publisher,
video_recorder). Kept as an escape hatch while the bag pipeline
beds in. In legacy mode the automator publishes /data/toggle_collect
on test start/stop so video_recorder + data_publisher react.

This launch file ONLY starts nodes not already covered by DEMO_DAY:
  - gps_publisher          (GPS fix data)
  - electrical_publisher   (voltage/current/power from INA226)
  - t000_automator         (A button start/stop control)
  - data_publisher         (legacy CSV — disabled by default)
  - video_recorder         (legacy camera+LiDAR MP4 — disabled by default)

It does NOT start odom, ZED, control, or joy — those are already
running from pre_slam.launch.py and run-zed.sh.
'''

def generate_launch_description():

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    enable_legacy_capture = DeclareLaunchArgument(
        'enable_legacy_capture',
        default_value='false',
        description=(
            'true → start the legacy per-sensor CSV + MP4 pipeline '
            '(data_publisher + video_recorder Nodes, base_automator '
            'CSV writes). false (default) → no automator-side data '
            'acquisition; ROS bag recording is HUD-driven via the R key.'
        ),
    )

    # Package directories
    autonav_testing_share = get_package_share_directory('autonav_automated_testing')
    electrical_share = FindPackageShare('autonav_electrical_publisher')

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

    # Data Publisher Node — legacy CSV path. Skipped in bag-only mode.
    data_publisher_node = Node(
        package='autonav_automated_testing',
        executable='data_publisher',
        name='data_publisher_node',
        output='screen',
        parameters=[{
            'topics_to_monitor': list(topics_to_monitor),
            'test_id': 't000',
            'publish_rate': 30.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('enable_legacy_capture')),
    )

    # GPS Handler Node — publishes GPS fix data to /gps_fix
    # (not started by any DEMO_DAY step)
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

    # Electrical Publisher Node — voltage/current/power from INA226
    # (not started by any DEMO_DAY step)
    electrical_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([electrical_share, 'launch', 'electrical_publisher.launch.py'])
        )
    )

    # Execute the test automator script (A button start/stop)
    automater_script_path = os.path.join(
        autonav_testing_share,
        'src',
        'automators',
        't000_automator.py'
    )
    test_automater = ExecuteProcess(
        cmd=[
            'python3', '-u',
            automater_script_path,
            '--ros-args',
            '-p',
            PythonExpression([
                "'use_sim_time:=' + '", LaunchConfiguration('use_sim_time'), "'"
            ]),
            '-p',
            PythonExpression([
                "'enable_legacy_capture:=' + '",
                LaunchConfiguration('enable_legacy_capture'),
                "'",
            ]),
        ],
        output='screen',
        name='t000_automater'
    )

    # Video recorder — legacy camera + LiDAR BEV MP4 path. Skipped in
    # bag-only mode.
    video_recorder_script_path = os.path.join(
        autonav_testing_share,
        'src',
        'video_recorder.py'
    )
    video_recorder = ExecuteProcess(
        cmd=[
            'python3', '-u',
            video_recorder_script_path,
            '--ros-args',
            '-p',
            PythonExpression([
                "'use_sim_time:=' + '", LaunchConfiguration('use_sim_time'), "'"
            ])
        ],
        output='screen',
        name='video_recorder',
        condition=IfCondition(LaunchConfiguration('enable_legacy_capture')),
    )

    return LaunchDescription([
        use_sim_time,
        enable_legacy_capture,
        # Nodes not covered by DEMO_DAY
        gps_handler_node,
        electrical_publisher_launch,
        # Data collection — only when legacy mode is on
        data_publisher_node,
        # Automator (always; owns the bag subprocess in bag-only mode)
        test_automater,
        # Video recording — only when legacy mode is on
        video_recorder
    ])
