from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bt_xml_path = PathJoinSubstitution([
        get_package_share_directory('slam'),
        'behavior_trees',
        'bt_lines_planner.xml'
    ])

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('slam'),
            'config',
            'nav2_lines_no_slam_params.yaml'
        ]),
        description='Path to Nav2 params'
    )

    auto_nav_goal = DeclareLaunchArgument(
        'auto_nav_goal',
        default_value='true',
        description='Send a NavigateToPose goal automatically on startup'
    )

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path,
            'default_nav_through_poses_bt_xml': bt_xml_path
        },
        convert_types=True
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('slam'),
                'launch',
                'dual_ekf_navsat.launch.py'
            ])
        ])
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'bt_navigator',
    ]

    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params]
    )

    navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )

    goal_payload = (
        "{pose: {header: {frame_id: base_link}, "
        "pose: {position: {x: 4.0, y: 0.0, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
    )

    send_goal = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose',
                    goal_payload
                ],
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('auto_nav_goal'))
    )

    return LaunchDescription([
        use_sim_time,
        nav2_params,
        auto_nav_goal,
        localization,
        planner,
        controller,
        navigator,
        manager,
        send_goal
    ])
