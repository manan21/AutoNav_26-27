import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


'''
Launch file for SLAM + Nav2 bringup.

Key ordering decisions:
  1. EKF (local odometry) starts first — SLAM toolbox needs odom->base_link.
  2. SLAM toolbox starts second — needs odom, publishes /map.
  3. map_padder starts third — needs /map, publishes /map_padded (transient_local).
  4. Nav2 (via navigation_launch.py) starts last via a TimerAction delay —
     this ensures map_padder has already latched /map_padded before the
     global costmap's StaticLayer subscribes. Without the delay, Nav2 starts
     before map_padder publishes its first message and the transient_local
     re-send only works if map_padder is already alive.

The SLAM config uses slam.yaml (NOT mapper_params_online_async.yaml) because
slam.yaml has the correct base_frame / scan_topic for this robot.
'''

def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    publish_period = DeclareLaunchArgument(
        'publish_period',
        default_value='0.02',
        description='SLAM map->odom TF publish period (s). 0.02 = 50 Hz.')

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('slam'),
            'config',
            'nav2_paramsv2.yaml'
        ]),
        description='Path to Nav2 parameters file')

    # ── Paths ─────────────────────────────────────────────────────────────
    pkg_share = FindPackageShare(package='slam').find('slam')
    slam_config      = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')

    # ── 1. Local EKF (odom -> base_link) ─────────────────────────────────
    # MUST be named 'ekf_node' — slam_toolbox looks for this exact name
    # when it reads the TF tree.
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            ekf_local_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('odometry/filtered', 'local_ekf/odom')]
    )

    # ── 2. SLAM Toolbox ───────────────────────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'transform_publish_period': LaunchConfiguration('publish_period'),
            }
        ]
    )

    # ── 3. Map Padder ─────────────────────────────────────────────────────
    # Subscribes to /map (transient_local from slam_toolbox) and publishes
    # /map_padded (also transient_local). Must be alive BEFORE Nav2 starts
    # so the StaticLayer receives the latched message on subscribe.
    map_padder = Node(
        package='map_padder',
        executable='map_padder_node',
        name='map_padder',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'tile_size_m': 1.0,
            'output_resolution': 0.10,
            # Subscribe to SLAM's map topic and republish as /map_padded
            'input_topic': '/map',
            'output_topic': '/map_padded',
        }]
    )

    # ── 4. Nav2 (delayed) ────────────────────────────────────────────────
    # Delayed 8 s so map_padder has time to receive /map from SLAM and
    # publish at least one /map_padded message before Nav2's StaticLayer
    # subscribes. Adjust the delay if SLAM takes longer to produce /map.
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('nav2_params'),
                }.items()
            )
        ]
    )

    # ── 5. GUI ready signal ───────────────────────────────────────────────
    gui_ready_emit = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 10 && echo "[GUI_READY] SLAM+Nav2"'],
        output='screen',
    )

    return LaunchDescription([
        # args
        use_sim_time,
        publish_period,
        nav2_params_arg,
        # nodes (in startup order)
        ekf_local,
        slam_toolbox,
        map_padder,
        nav2,
        gui_ready_emit,
    ])