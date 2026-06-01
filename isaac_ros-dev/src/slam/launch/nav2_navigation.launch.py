"""Bond-timeout-patched Nav2 navigation bringup (thin wrapper).

Why this file exists
--------------------
``run-nav2.sh`` previously launched the stock
``nav2_bringup/navigation_launch.py`` directly. That launch file builds
``lifecycle_manager_navigation`` with ONLY these parameters::

    parameters=[{'use_sim_time': ...}, {'autostart': ...}, {'node_names': ...}]

i.e. it does NOT load ``params_file`` for the lifecycle manager, so a
``bond_timeout`` placed in ``nav2_paramsv2.yaml`` is silently ignored.

On this robot's Jetson (Orin Nano, 6 cores, 15W power mode) the nav2 stack
is CPU-starved while a mission runs. With the default 4 s ``bond_timeout``
the lifecycle manager's ``checkBondConnections`` watchdog decides the
(merely slow) servers have died and deactivates planner_server /
controller_server / bt_navigator / behavior_server / velocity_smoother in a
restart loop -- so no path is ever produced even though the goal is
received.

This wrapper raises ``bond_timeout`` to 20 s via ``SetParameter`` (the only
way to reach the lifecycle manager node inside the stock launch without
forking the whole file). ``bond_timeout`` is an unused override on the
server nodes, which ignore it. ``use_composition`` is forced ``False``
(already this robot's mode -- each server runs as its own process) so
``SetParameter`` applies to the regular Node actions.

Revert: point ``run-nav2.sh`` back at ``nav2_bringup navigation_launch.py``
and delete this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    stock_launch = os.path.join(
        nav2_bringup_dir, 'launch', 'navigation_launch.py')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Absorbed only for run-nav2.sh interface compatibility. The stock
        # navigation_launch.py never declared this argument, so it has had
        # no effect; the active behavior tree comes from bt_navigator's
        # default_nav_to_pose_bt_xml in nav2_paramsv2.yaml. Declared here so
        # passing it does not emit an "undeclared argument" warning.
        DeclareLaunchArgument('default_bt_xml_filename', default_value=''),

        GroupAction([
            # bond_timeout 4.0 s (nav2 default) -> 20.0 s. See module docstring.
            SetParameter(name='bond_timeout', value=20.0),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stock_launch),
                launch_arguments={
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                    'use_composition': 'False',
                }.items(),
            ),
        ]),
    ])
