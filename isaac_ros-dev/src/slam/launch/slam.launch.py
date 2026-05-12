import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
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

    # ── GPS fusion toggle ────────────────────────────────────────────
    # When true (default), the Map EKF (ekf_global) and
    # navsat_transform_node come up alongside slam_toolbox, fusing
    # /gps_fix into /global_ekf/odom as an XY-only anchor. Set false
    # at launch time if a GPS receiver is misbehaving and you want to
    # fall back to a SLAM-only Map EKF without rebuilding.
    enable_gps_fusion = DeclareLaunchArgument(
        'enable_gps_fusion',
        default_value='true',
        description='Run ekf_global + navsat_transform_node so GPS '
                    'contributes XY-only corrections to /global_ekf/odom.')

    # ── Paths ─────────────────────────────────────────────────────────────
    pkg_share = FindPackageShare(package='slam').find('slam')
    slam_config       = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_local_config  = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    # ── 0. Local EKF (odom -> base_link) ─────────────────────────────────
    # robot_localization can rotate IMU measurements into base_link using TF,
    # so the EKF consumes the raw IMU topic directly.
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

    # ── 2.25 Map EKF (map -> base_link, NO TF publish) ──────────────────
    # Fuses slam_toolbox /pose, the Local EKF output, and (when
    # enable_gps_fusion is true) /odometry/gps from navsat_transform.
    # Emits /global_ekf/odom for downstream consumers. publish_tf is
    # OFF in ekf_global.yaml because slam_toolbox already publishes
    # the map -> odom transform.
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[
            ekf_global_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('odometry/filtered', '/global_ekf/odom')],
    )

    # ── 2.4 navsat_transform_node (only when enable_gps_fusion) ─────────
    # Converts /gps_fix → /odometry/gps in the map frame. With
    # use_odometry_yaw=true the heading reference comes from the Map
    # EKF itself (no magnetometer on this robot), so the first GPS
    # corrections may be rotationally imprecise until the robot has
    # moved enough for the EKF yaw to be meaningful. The XY-only mask
    # in ekf_global.yaml limits how much a bad early fix can pull the
    # estimate. delay=3.0 lets the EKF stabilize before the first GPS
    # measurement is forwarded.
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frequency': 30.0,
            'delay': 3.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'broadcast_cartesian_transform': False,
            'publish_filtered_gps': True,
            'use_odometry_yaw': True,
            'wait_for_datum': False,
        }],
        remappings=[
            ('gps/fix',          '/gps_fix'),
            ('imu',              '/sick_scansegment_xd/imu'),
            ('odometry/filtered', '/global_ekf/odom'),
            ('odometry/gps',     '/odometry/gps'),
            ('gps/filtered',     '/gps/filtered'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_gps_fusion')),
    )

    # ── 2.5 PCA PointCloud2 → LaserScan converter ────────────────────────
    # autonav_detection::grade_detector publishes /scan_pca_filtered_points
    # (PointCloud2, obstacles only — drivable ground / ramps already
    # filtered out via PCA grade classification). Nav2's obstacle_layer
    # in nav2_paramsv2.yaml is configured to consume the 2-D LaserScan
    # /scan_pca_filtered, so we collapse the 3-D cloud once here and let
    # both local and global costmaps share the result.
    #
    # target_frame=base_link projects each point into base_link first;
    # then the (min_height, max_height) window in base_link selects which
    # heights count. -0.10 m → 1.50 m covers everything from just below
    # the wheels to ~1.5 m above the chassis, which is more than enough
    # for any AutoNav obstacle. angle_increment 0.0087 rad ≈ 0.5°.
    pca_pc2_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pca_cloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time':    LaunchConfiguration('use_sim_time'),
            'target_frame':   'base_link',
            'min_height':     -0.10,
            'max_height':      1.50,
            'angle_min':      -3.141592,
            'angle_max':       3.141592,
            'angle_increment': 0.0087,
            'scan_time':       0.1,
            'range_min':       0.30,
            'range_max':      25.0,
            'use_inf':         True,
        }],
        remappings=[
            ('cloud_in', '/scan_pca_filtered_points'),
            ('scan',     '/scan_pca_filtered'),
        ],
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
        enable_gps_fusion,
        # nodes (in startup order)
        ekf_local,
        slam_toolbox,
        ekf_global,
        navsat_transform,
        pca_pc2_to_scan,
        map_padder,
        nav2,
        gui_ready_emit,
    ])
