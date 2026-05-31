#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_share(package: str) -> str:
    return get_package_share_directory(package)


def _default_course_config() -> str:
    return os.path.join(
        _package_share("igvc_competition_sim"),
        "config",
        "igvc_competition_compact.yaml",
    )


def _default_world() -> str:
    return os.path.join(
        _package_share("igvc_competition_sim"),
        "worlds",
        "igvc_competition_compact.sdf",
    )


def _default_bt_xml() -> str:
    try:
        candidate = Path(_package_share("slam")) / "behavior_trees" / "bt_nav.xml"
        if candidate.is_file():
            return str(candidate)
    except Exception:
        pass
    autonav_repo = os.environ.get(
        "AUTONAV_REPO", str(Path.home() / "code/git/AutoNav_25-26"))
    return str(Path(autonav_repo)
               / "isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml")


def _load_robot_description() -> str:
    candidates = []
    try:
        candidates.append(
            Path(_package_share("bringup")) / "description" / "shogi.urdf")
    except Exception:
        pass
    candidates.append(
        Path(os.environ.get(
            "AUTONAV_REPO",
            str(Path.home() / "code/git/AutoNav_25-26"),
        )) / "isaac_ros-dev/src/bringup/description/shogi.urdf",
    )
    for path in candidates:
        if path.is_file():
            return path.read_text(encoding="utf-8")
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError("Could not find shogi.urdf. Searched:\n  " + searched)


def _gazebo_process(context, *args, **kwargs):
    enabled = LaunchConfiguration("launch_gazebo").perform(context).lower()
    if enabled not in ("1", "true", "yes", "on"):
        return []
    world = LaunchConfiguration("world").perform(context)
    server_only = (
        LaunchConfiguration("gazebo_server_only").perform(context).lower()
        in ("1", "true", "yes", "on")
    )
    server_args = ["-s"] if server_only else []
    gz = shutil.which("gz")
    ign = shutil.which("ign")
    if gz:
        cmd = [gz, "sim", *server_args, "-r", world]
    elif ign:
        cmd = [ign, "gazebo", *server_args, "-r", world]
    else:
        cmd = ["ign", "gazebo", *server_args, "-r", world]
    return [ExecuteProcess(cmd=cmd, output="screen")]


def _robot_state_publisher(context, *args, **kwargs):
    _ = context
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": _load_robot_description(),
                "use_sim_time": True,
            }],
        )
    ]


def _nav2_params_with_bt(params_file: str, bt_xml: str) -> str:
    source = Path(params_file)
    text = source.read_text(encoding="utf-8")
    for key in ("default_nav_to_pose_bt_xml",
                "default_nav_through_poses_bt_xml"):
        text = re.sub(
            rf"({key}:\s*).+",
            rf'\1"{bt_xml}"',
            text,
        )
    digest = hashlib.sha1(
        (str(source) + "\n" + bt_xml).encode("utf-8")).hexdigest()[:12]
    output = Path(tempfile.gettempdir()) / f"igvc_nav2_params_{digest}.yaml"
    output.write_text(text, encoding="utf-8")
    return str(output)


def _nav2_process(context, *args, **kwargs):
    enabled = LaunchConfiguration("launch_nav").perform(context).lower()
    if enabled not in ("1", "true", "yes", "on"):
        return []
    nav2_launch = os.path.join(
        _package_share("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )
    params_file = _nav2_params_with_bt(
        LaunchConfiguration("nav2_params").perform(context),
        LaunchConfiguration("bt_xml").perform(context),
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": "true",
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    course_config = LaunchConfiguration("course_config")
    launch_gazebo = LaunchConfiguration("launch_gazebo")
    launch_bridge = LaunchConfiguration("launch_bridge")
    ground_truth_pca = LaunchConfiguration("ground_truth_pca")
    fallback_integrate_cmd = LaunchConfiguration("fallback_integrate_cmd")

    nav2_params = LaunchConfiguration("nav2_params")
    bt_xml = LaunchConfiguration("bt_xml")

    detection_launch = os.path.join(
        _package_share("autonav_detection"),
        "launch",
        "detection.launch.py",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="igvc_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/shogi/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        condition=IfCondition(launch_bridge),
    )

    harness = Node(
        package="igvc_competition_sim",
        executable="igvc_sensor_harness",
        name="igvc_sensor_harness",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "course_config": course_config,
            "fallback_integrate_cmd": fallback_integrate_cmd,
            "publish_ground_truth_pca": ground_truth_pca,
        }],
    )

    monitor = Node(
        package="igvc_competition_sim",
        executable="igvc_course_monitor",
        name="igvc_course_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "course_config": course_config,
        }],
    )

    detection_real_pca = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(detection_launch),
        launch_arguments={
            "enable_line": "false",
            "enable_grade": "true",
            "enable_lidar_line": "true",
        }.items(),
        condition=UnlessCondition(ground_truth_pca),
    )
    detection_ground_truth_pca = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(detection_launch),
        launch_arguments={
            "enable_line": "false",
            "enable_grade": "false",
            "enable_lidar_line": "true",
        }.items(),
        condition=IfCondition(ground_truth_pca),
    )

    pca_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pca_cloud_to_laserscan",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "min_height": -0.10,
            "max_height": 1.50,
            "angle_min": -1.5708,
            "angle_max": 1.5708,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.30,
            "range_max": 25.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/scan_pca_filtered_points"),
            ("scan", "/scan_pca_filtered"),
        ],
    )

    pca_scan_clear = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pca_cloud_to_laserscan_clear",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "min_height": -0.10,
            "max_height": 1.50,
            "angle_min": -1.2217,
            "angle_max": 1.2217,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.30,
            "range_max": 25.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/scan_pca_filtered_points"),
            ("scan", "/scan_pca_filtered_clear"),
        ],
    )

    gps_handler = Node(
        package="gps_waypoint_handler",
        executable="gps_handler_node",
        name="gps_handler_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("course_config", default_value=_default_course_config()),
        DeclareLaunchArgument("world", default_value=_default_world()),
        DeclareLaunchArgument("launch_gazebo", default_value="true"),
        DeclareLaunchArgument("gazebo_server_only", default_value="true"),
        DeclareLaunchArgument("launch_bridge", default_value="true"),
        DeclareLaunchArgument("launch_nav", default_value="true"),
        DeclareLaunchArgument("ground_truth_pca", default_value="false"),
        DeclareLaunchArgument("fallback_integrate_cmd", default_value="true"),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(
                _package_share("slam"), "config", "nav2_paramsv2.yaml"),
        ),
        DeclareLaunchArgument("bt_xml", default_value=_default_bt_xml()),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.pathsep.join([
                _package_share("igvc_competition_sim"),
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            ]),
        ),
        OpaqueFunction(function=_gazebo_process),
        bridge,
        OpaqueFunction(function=_robot_state_publisher),
        harness,
        monitor,
        detection_real_pca,
        detection_ground_truth_pca,
        pca_scan,
        pca_scan_clear,
        gps_handler,
        OpaqueFunction(function=_nav2_process),
    ])
