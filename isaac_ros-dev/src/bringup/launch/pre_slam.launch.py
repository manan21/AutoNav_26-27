import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction
)

from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown
)


# joy | core | lidar
#  |
#  v
#control  
#  | 
#  v
# odom ---------> SLAM 


def generate_launch_description():

    # joy | core | 

    sick_pkg = os.path.join(get_package_share_directory('sick_scan_xd'), 'launch', 'sick_multiscan.launch.py')
    sick_pkg = PathJoinSubstitution([FindPackageShare('sick_scan_xd'), 'launch', 'sick_multiscan.launch.py'])
    core_pkg = PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'core_bringup.launch.py'])
    control_pkg = PathJoinSubstitution([FindPackageShare('control'), 'launch', 'control_dev.launch.py'])
    
    

    sick_args = {
        'hostname':'192.168.0.1',
        'udp_receiver_ip':'192.168.0.2',
        'publish_frame_id':'lidar_footprint',
        'tf_publish_rate':'0'
    }


    sick = IncludeLaunchDescription(
             sick_pkg, launch_arguments=sick_args.items()
    )
    core = IncludeLaunchDescription(
            core_pkg ,
    )

    # joy_node is provided by control_dev.launch.py (the file the
    # ``control`` IncludeLaunchDescription below pulls in). Launching
    # it here too produced two /joy nodes under the same name, which
    # was the "WARNING: there are nodes in the graph that share an
    # exact name" the operator was seeing alongside the duplicate
    # Nav2 nodes. control_dev fires 1 s after this launch starts via
    # control_event below — joy is up before any consumer needs it.

    control = IncludeLaunchDescription(
        control_pkg
    )

    odom = Node(
        package='odom_handler',
        executable='wheel_odometry_publisher',
        name='wheel_odom'
    )

    # imu_cov_inflator is a sensor pre-processor that takes the SICK
    # multiScan's raw /sick_scansegment_xd/imu (which ships with
    # all-zero covariance) and republishes it as /imu_inflated with
    # sane covariance values so downstream EKF and grade-compensation
    # consumers can weight it correctly. Lives here in pre_slam (not
    # in slam.launch.py as it used to) so:
    #   - Phase D grade compensation in control_node works for
    #     bench-test in manual mode without firing the full slam stack.
    #   - The HUD's IMU display gets data immediately on bring-up.
    #   - Architecturally it belongs alongside the lidar driver — it's
    #     a sensor-pipeline node, not a nav2 node.
    imu_cov_inflator = Node(
        package='imu_cov_inflator',
        executable='imu_inflator_node',
        name='imu_cov_inflator',
        output='screen',
    )

    control_event = TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg='Control node booting up...'),
                control
            ]

    )
    odom_event = TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='Odom node booting up...'),
                odom
            ]

    )


    return LaunchDescription([
        core,
        imu_cov_inflator,
        control_event,
        odom_event

    ])

    
    

