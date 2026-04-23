from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    camera_model = DeclareLaunchArgument(
        "camera_model",
        default_value="zed2i",
        description="ZED camera model to launch",
    )
    hostname = DeclareLaunchArgument(
        "hostname",
        default_value="192.168.0.1",
        description="IP address of the SICK lidar",
    )
    udp_receiver_ip = DeclareLaunchArgument(
        "udp_receiver_ip",
        default_value="192.168.0.2",
        description="IP address of this machine for the SICK lidar UDP receiver",
    )
    publish_period = DeclareLaunchArgument(
        "publish_period",
        default_value="0.02",
        description="SLAM map-to-odom transform publish period",
    )
    max_laserscan_range = DeclareLaunchArgument(
        "max_laserscan_range",
        default_value="10.0",
        description="Maximum range in meters for the native SICK LaserScan publisher",
    )
    nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=PathJoinSubstitution([
            get_package_share_directory("slam"),
            "config",
            "nav2_paramsv2.yaml",
        ]),
        description="Nav2 parameter file for demo-day navigation",
    )

    bringup_share = FindPackageShare("bringup")
    slam_share = FindPackageShare("slam")

    pre_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "pre_slam.launch.py"])
        )
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
        ),
        launch_arguments={
            "camera_model": LaunchConfiguration("camera_model"),
            "hostname": LaunchConfiguration("hostname"),
            "udp_receiver_ip": LaunchConfiguration("udp_receiver_ip"),
            "max_laserscan_range": LaunchConfiguration("max_laserscan_range"),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "slam.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_period": LaunchConfiguration("publish_period"),
            "nav2_params": LaunchConfiguration("nav2_params"),
        }.items(),
    )

    line_detection = Node(
        package="line_detection",
        executable="line_detector",
        name="line_detection_node",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "camera_topic": "/zed/zed_node/rgb/color/rect/image",
            "depth_camera_topic": "/zed/zed_node/depth/depth_registered",
            "camera_info_topic": "/zed/zed_node/rgb/color/rect/camera_info",
            "line_points_topic": "/line_points",
            "target_frame": "odom",
            "enable_timer": True,
            "publish_interval_ms": 250,
            "max_rgb_depth_delta_ms": 120,
            "tf_lookup_timeout_ms": 100,
            "line_hold_timeout_ms": 2500,
            "line_memory_resolution_m": 0.05,
            "line_memory_max_points": 20000,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "nav.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "nav2_params": LaunchConfiguration("nav2_params"),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time,
        camera_model,
        hostname,
        udp_receiver_ip,
        publish_period,
        max_laserscan_range,
        nav2_params,
        pre_slam,
        sensors,
        slam,
        line_detection,
        nav2,
    ])
