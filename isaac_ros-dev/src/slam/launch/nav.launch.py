from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

# Launch Nav2 with a package-share BT path so the runtime does not depend
# on a machine-specific absolute workspace location.

def generate_launch_description():

    bt_xml_path = PathJoinSubstitution([
        get_package_share_directory('slam'),
        'behavior_trees',
        'bt_2.xml'
    ])

    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true')

    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('slam'),
            'config',
            'nav2_lines_params.yaml'
        ]),
        description='Path to your custom Nav2 parameters file'
    )

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path,
            'default_nav_through_poses_bt_xml': bt_xml_path,
        },
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': configured_params,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time,
        nav2_params,
        nav2,
    ])
