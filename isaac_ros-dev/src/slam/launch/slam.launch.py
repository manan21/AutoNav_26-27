import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


# Workspace-relative path to the readiness helpers. Both shell scripts and
# this launch file rely on this single copy.
WAIT_FOR_TOPIC_PY = '/autonav/isaac_ros-dev/config/lib/wait_for_topic.py'


'''
Launch file for slam launch. This version is not GPU dependent.

This system creates map and performs all localization, and publishes map->odom->base_link.

The transforms published are the requirements for the rest of NAV2 to work properly. 
'''

def generate_launch_description():
        # Launch Arguments
    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true')

    publish_period = DeclareLaunchArgument(
        'publish_period',
        # 0.02 if you want to publish
        default_value='0.02',
        description="if you want SLAM to publish map->odom... (sim yes real no)"
    )
    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('slam'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Path to your custom Nav2 parameters file'
    )


    
    # rest in peace ... i will eternalize it in these comments
    # magic_spell = lambda x : 0.02 if x else 0.00
                                            

    pkg_share = FindPackageShare(package='slam').find('slam')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local_sim.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
        
        # 1. LiDAR PointCloud to LaserScan Conversion

        # deps for Lidar 2 pointcloud (remove)
        #sudo apt update
        #sudo apt install ros-humble-vision-msgs ros-humble-tf2-sensor-msgs
        # sudo apt install ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-sensor-msgs


        # !!!
    # 2. SLAM Toolbox (Online Async)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        #remappings=[
        #    ('/scan', '/scan_fullframe')
        #],
        parameters=[slam_config, {"use_sim_time": LaunchConfiguration('use_sim_time'),
                                  "transform_publish_period": LaunchConfiguration('publish_period')
                                  }]
    )
   
    # 2. Include the Nav2 bringup launch file with your params
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'), 
            # 'params_file': LaunchConfiguration('nav2_params')
        }.items()
    )
    
    '''
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    '''

    # 3. Local EKF (odom -> base_link)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node', # this has to be called ekf_node or slam toolbox freaks the fuck out
        output='screen',
        parameters=[ekf_local_config , {"use_sim_time": LaunchConfiguration('use_sim_time')}],
        remappings=[('odometry/filtered', 'local_ekf/odom')]
    )

    # 4. GPS Transformation Node
    gps_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            {'zero_altitude': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', 'global_ekf/odom')
        ]
    )


    # 5. Global EKF (map -> odom)
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_config , {"use_sim_time": LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('odometry/filtered', 'global_ekf/odom')
        ]
    )

    # Pad the SLAM map so the global costmap always contains the robot + GPS waypoints
    map_padder = Node(
        package='map_padder',
        executable='map_padder_node',
        name='map_padder',
        output='screen',
        parameters=[{
            'tile_size_m': 1.0,
            'output_resolution': 0.10,
        }]
    )

    # Block slam_toolbox + map_padder until /scan_fullframe is actually
    # publishing. Without this gate, slam_toolbox can latch a "no data"
    # state if it starts before the SICK driver / network is up, and
    # never recovers even after scans begin flowing.
    wait_for_scan = ExecuteProcess(
        cmd=[
            'python3', WAIT_FOR_TOPIC_PY,
            '/scan_fullframe',
            '--type', 'sensor_msgs/msg/LaserScan',
            '--qos', 'sensor',
            '--timeout', '120',
        ],
        output='screen',
    )
    start_when_scan_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan,
            on_exit=[slam_toolbox, map_padder],
        )
    )

    # Once /map_padded has its first sample, print the GUI readiness sentinel
    # so the HUD can flip the SLAM dot green and start the next queued device.
    gui_ready_emit = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'python3 {WAIT_FOR_TOPIC_PY} /map_padded '
            f'--type nav_msgs/msg/OccupancyGrid '
            f'--qos transient_local --timeout 120 '
            f'&& echo "[GUI_READY] SLAM" '
            f'|| echo "[GUI_READY_TIMEOUT] SLAM" >&2',
        ],
        output='screen',
    )

    return LaunchDescription([
        # params
        publish_period,
        use_sim_time,
        nav2_params,
        # nodes — ekf_local has no lidar dependency, start it immediately
        ekf_local,
        # gate slam_toolbox + map_padder on /scan_fullframe
        wait_for_scan,
        start_when_scan_ready,
        # readiness handshake for the HUD (parallel; emits when /map_padded fires)
        gui_ready_emit,
        #gps_transform,
        #ekf_global,
        #nav2

    ])

