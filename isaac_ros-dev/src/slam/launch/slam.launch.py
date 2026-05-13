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
Launch file for SLAM bringup (Nav2 launches separately).

Key ordering decisions:
  1. EKF (local odometry) starts first — SLAM toolbox needs odom->base_link.
  2. SLAM toolbox starts second — needs odom, publishes /map.
  3. Map EKF + navsat_transform follow once /pose and /local_ekf/odom are
     wired up; navsat_transform feeds /odometry/gps into the Map EKF as
     an XY-only anchor (see ekf_global.yaml).
  4. map_padder needs /map, publishes /map_padded (transient_local).
  5. Nav2 is NOT launched here — it is started by the GUI's NAV2 button
     (./config/run-nav2.sh → ros2 launch nav2_bringup navigation_launch.py).
     Launching it from both sites produced two lifecycle_manager_navigation
     processes under the same node name; they raced and neither finished
     configuring its costmaps, which silently held map → odom back and
     left /goal_pose stuck at (0, 0). The [GUI_READY] sentinel below
     intentionally waits ~10 s so map_padder has time to receive the
     first /map from slam_toolbox before the GUI's launch queue
     graduates and fires the NAV2 button.

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
        default_value='0.0',
        description='SLAM map->odom TF publish period (s). 0.02 = 50 Hz, '
                    '0.0 disables (static_transform_publisher owns the TF).')

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

    # Inflates /sick_scansegment_xd/imu's all-zero covariance so ekf_local can fuse yaw rate without over-trusting it.
    imu_cov_inflator = Node(
        package='imu_cov_inflator',
        executable='imu_inflator_node',
        name='imu_cov_inflator',
        output='screen',
    )

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

    # Static identity map->odom owns the TF; slam_toolbox is silent on TF.
    map_to_odom_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
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
        # Same gate as navsat_transform — setting
        # enable_gps_fusion:=false disables the entire Map EKF path
        # (both ekf_global and navsat_transform) so the operator can
        # A/B test against the pre-fusion baseline without rebuilding.
        condition=IfCondition(LaunchConfiguration('enable_gps_fusion')),
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
    # Asymmetric marking vs clearing FOV: mark anything the lidar sees
    # in the full 180° forward view, but only clear (raytrace-free) the
    # central 160° (±80°). The outer 10° wedge on each side is "trust
    # the sensor's hits, don't trust its claim that a cell is free" —
    # the lidar's beam density / angular accuracy is worst at the
    # extremes, so we don't let those rays raytrace-clear stored marks.
    # Two converters share the same input pointcloud:
    #   /scan_pca_filtered      → 180°, marking source
    #   /scan_pca_filtered_clear→ 160°, clearing source
    # The local obstacle_layer in nav2_paramsv2.yaml lists both as
    # observation_sources, with marking/clearing flags split.
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
            'angle_min':      -1.5708,   # -90° (full 180° marking)
            'angle_max':       1.5708,   # +90°
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

    pca_pc2_to_scan_clear = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pca_cloud_to_laserscan_clear',
        output='screen',
        parameters=[{
            'use_sim_time':    LaunchConfiguration('use_sim_time'),
            'target_frame':   'base_link',
            'min_height':     -0.10,
            'max_height':      1.50,
            'angle_min':      -1.2217,   # -70° (140° clearing range)
            'angle_max':       1.2217,   # +70°
            'angle_increment': 0.0087,
            'scan_time':       0.1,
            'range_min':       0.30,
            'range_max':      25.0,
            'use_inf':         True,
        }],
        remappings=[
            ('cloud_in', '/scan_pca_filtered_points'),
            ('scan',     '/scan_pca_filtered_clear'),
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

    # ── 4. Nav2 ─────────────────────────────────────────────────────────
    # Nav2 is launched separately by the GUI's NAV2 button (which runs
    # ``./config/run-nav2.sh`` → ``ros2 launch nav2_bringup
    # navigation_launch.py``). Launching it here too was the cause of
    # two ``lifecycle_manager_navigation`` / ``bt_navigator`` /
    # ``planner_server`` / ``controller_server`` instances racing under
    # the same node names — the manifest of "WARNING: there are nodes
    # in the graph that share an exact name" and the symptom of
    # /goal_pose stalling at (0, 0) because slam_toolbox never
    # graduated to publishing ``map → odom``. Single Nav2 owner now:
    # the GUI button. SLAM's [GUI_READY] sentinel below still fires
    # ~10 s after launch so map_padder has time to receive the first
    # /map from slam_toolbox before the NAV2 button is allowed to fire
    # next in the GUI's launch queue.

    # ── 5. GUI ready signal ───────────────────────────────────────────────
    gui_ready_emit = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 10 && echo "[GUI_READY] SLAM"'],
        output='screen',
    )

    return LaunchDescription([
        # args
        use_sim_time,
        publish_period,
        nav2_params_arg,
        enable_gps_fusion,
        # nodes (in startup order)
        imu_cov_inflator,
        ekf_local,
        slam_toolbox,
        map_to_odom_static,
        ekf_global,
        navsat_transform,
        pca_pc2_to_scan,
        pca_pc2_to_scan_clear,
        map_padder,
        gui_ready_emit,
    ])
