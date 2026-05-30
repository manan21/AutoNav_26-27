from __future__ import annotations

import argparse
import math
from pathlib import Path
import re

from .course import Course, DEFAULT_COURSE_CONFIG, course_bounds, load_course


def _clean_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_") or "model"


def _material(name: str, rgba: tuple[float, float, float, float]) -> str:
    r, g, b, a = rgba
    return (
        f"<material><ambient>{r} {g} {b} {a}</ambient>"
        f"<diffuse>{r} {g} {b} {a}</diffuse>"
        f"<specular>0.05 0.05 0.05 1</specular></material>"
    )


def _box_visual_model(name: str,
                      pose: tuple[float, float, float, float, float, float],
                      size: tuple[float, float, float],
                      rgba: tuple[float, float, float, float],
                      collide: bool = False,
                      static: bool = True) -> str:
    collision = ""
    if collide:
        collision = (
            "<collision name='collision'><geometry><box>"
            f"<size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size>"
            "</box></geometry></collision>"
        )
    return f"""
    <model name='{_clean_name(name)}'>
      <static>{1 if static else 0}</static>
      <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
      <link name='link'>
        {collision}
        <visual name='visual'>
          <geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>
          {_material('mat', rgba)}
        </visual>
      </link>
    </model>"""


def _tape_model(name: str,
                start: tuple[float, float],
                end: tuple[float, float],
                width_m: float) -> str:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = max(0.01, math.hypot(dx, dy))
    yaw = math.atan2(dy, dx)
    cx = 0.5 * (start[0] + end[0])
    cy = 0.5 * (start[1] + end[1])
    return _box_visual_model(
        name,
        (cx, cy, 0.011, 0.0, 0.0, yaw),
        (length, width_m, 0.012),
        (0.98, 0.98, 0.98, 1.0),
        collide=False,
    )


def _cylinder_model(name: str,
                    x: float,
                    y: float,
                    radius: float,
                    height: float,
                    rgba: tuple[float, float, float, float],
                    collide: bool) -> str:
    collision = ""
    if collide:
        collision = (
            "<collision name='collision'><pose>0 0 "
            f"{height * 0.5:.4f} 0 0 0</pose><geometry><cylinder>"
            f"<radius>{radius:.4f}</radius><length>{height:.4f}</length>"
            "</cylinder></geometry></collision>"
        )
    return f"""
    <model name='{_clean_name(name)}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name='link'>
        {collision}
        <visual name='visual'>
          <pose>0 0 {height * 0.5:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>
          {_material('mat', rgba)}
        </visual>
      </link>
    </model>"""


def _ramp_model(course: Course) -> str:
    out: list[str] = []
    for ramp in course.ramps:
        run = ramp.end_x_m - ramp.start_x_m
        length = math.hypot(run, ramp.rise_m)
        pitch = -math.atan2(ramp.rise_m, run)
        out.append(_box_visual_model(
            ramp.name,
            (
                0.5 * (ramp.start_x_m + ramp.end_x_m),
                ramp.center_y_m,
                0.5 * ramp.rise_m,
                0.0,
                pitch,
                0.0,
            ),
            (length, ramp.width_m, 0.08),
            (0.45, 0.45, 0.42, 1.0),
            collide=True,
        ))
    return "\n".join(out)


def _robot_model(course: Course) -> str:
    robot = course.robot
    track = robot.wheel_track_m
    radius = robot.wheel_radius_m
    z0 = radius
    return f"""
    <model name='shogi'>
      <pose>{course.start.x:.4f} {course.start.y:.4f} {z0:.4f} 0 0 {course.start.yaw:.6f}</pose>
      <link name='base_link'>
        <inertial>
          <mass>90.0</mass>
          <inertia>
            <ixx>4.0</ixx><iyy>8.0</iyy><izz>9.0</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name='body_collision'>
          <pose>{robot.base_link_to_nav_center_m:.4f} 0 0.1800 0 0 0</pose>
          <geometry><box><size>1.0900 0.8200 0.2600</size></box></geometry>
        </collision>
        <visual name='body_visual'>
          <pose>{robot.base_link_to_nav_center_m:.4f} 0 0.1800 0 0 0</pose>
          <geometry><box><size>1.0900 0.8200 0.2600</size></box></geometry>
          {_material('body', (0.22, 0.28, 0.36, 1.0))}
        </visual>
        <sensor name='zed2i_rgbd' type='rgbd_camera'>
          <pose>0.657600 0.009075 0.307230 0 0.349070 0</pose>
          <always_on>1</always_on>
          <update_rate>15</update_rate>
          <visualize>false</visualize>
          <topic>/igvc_sim/zed</topic>
          <camera>
            <horizontal_fov>1.918862</horizontal_fov>
            <image>
              <width>960</width>
              <height>540</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.10</near>
              <far>15.0</far>
            </clip>
          </camera>
        </sensor>
      </link>
      <link name='left_wheel_link'>
        <pose>0 {track * 0.5:.5f} 0 1.570796 0 0</pose>
        <inertial><mass>3.0</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name='collision'><geometry><cylinder><radius>{radius:.5f}</radius><length>0.1000</length></cylinder></geometry></collision>
        <visual name='visual'><geometry><cylinder><radius>{radius:.5f}</radius><length>0.1000</length></cylinder></geometry>{_material('wheel', (0.03, 0.03, 0.03, 1.0))}</visual>
      </link>
      <link name='right_wheel_link'>
        <pose>0 {-track * 0.5:.5f} 0 1.570796 0 0</pose>
        <inertial><mass>3.0</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name='collision'><geometry><cylinder><radius>{radius:.5f}</radius><length>0.1000</length></cylinder></geometry></collision>
        <visual name='visual'><geometry><cylinder><radius>{radius:.5f}</radius><length>0.1000</length></cylinder></geometry>{_material('wheel', (0.03, 0.03, 0.03, 1.0))}</visual>
      </link>
      <joint name='Left_Wheel' type='revolute'>
        <parent>base_link</parent>
        <child>left_wheel_link</child>
        <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <joint name='Right_Wheel' type='revolute'>
        <parent>base_link</parent>
        <child>right_wheel_link</child>
        <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <plugin filename='ignition-gazebo-diff-drive-system' name='ignition::gazebo::systems::DiffDrive'>
        <left_joint>Left_Wheel</left_joint>
        <right_joint>Right_Wheel</right_joint>
        <wheel_separation>{track:.5f}</wheel_separation>
        <wheel_radius>{radius:.5f}</wheel_radius>
        <topic>/cmd_vel_gazebo</topic>
        <odom_topic>/model/shogi/odometry</odom_topic>
        <tf_topic>/model/shogi/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>50</odom_publish_frequency>
        <max_linear_acceleration>1.0</max_linear_acceleration>
        <max_angular_acceleration>2.0</max_angular_acceleration>
      </plugin>
    </model>"""


def generate_world(course: Course) -> str:
    min_x, min_y, max_x, max_y = course_bounds(course, margin_m=8.0)
    ground_size_x = max(60.0, max_x - min_x)
    ground_size_y = max(30.0, max_y - min_y)
    ground_x = 0.5 * (min_x + max_x)
    ground_y = 0.5 * (min_y + max_y)

    models: list[str] = []
    models.append(_box_visual_model(
        "asphalt_ground",
        (ground_x, ground_y, -0.025, 0.0, 0.0, 0.0),
        (ground_size_x, ground_size_y, 0.05),
        (0.16, 0.16, 0.15, 1.0),
        collide=True,
    ))
    for tape in course.tapes:
        models.append(_tape_model(tape.name, tape.start, tape.end, tape.width_m))
    for obstacle in course.obstacles:
        color = (0.95, 0.32, 0.05, 1.0)
        if obstacle.kind == "post":
            color = (0.25, 0.25, 0.25, 1.0)
        models.append(_cylinder_model(
            obstacle.name,
            obstacle.center[0],
            obstacle.center[1],
            obstacle.radius_m,
            obstacle.height_m,
            color,
            collide=True,
        ))
    for pothole in course.potholes:
        models.append(_cylinder_model(
            pothole.name,
            pothole.center[0],
            pothole.center[1],
            pothole.radius_m,
            0.012,
            (0.96, 0.96, 0.90, 1.0),
            collide=False,
        ))
    models.append(_ramp_model(course))
    models.append(_robot_model(course))

    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <world name='igvc_competition'>
    <plugin filename='ignition-gazebo-physics-system' name='ignition::gazebo::systems::Physics'/>
    <plugin filename='ignition-gazebo-user-commands-system' name='ignition::gazebo::systems::UserCommands'/>
    <plugin filename='ignition-gazebo-scene-broadcaster-system' name='ignition::gazebo::systems::SceneBroadcaster'/>
    <light name='sun' type='directional'>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.8 0.8 0.75 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.35 0.15 -0.92</direction>
    </light>
    <gravity>0 0 -9.80665</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.70 0.72 0.75 1</background>
      <shadows>true</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{course.datum_latitude_deg:.8f}</latitude_deg>
      <longitude_deg>{course.datum_longitude_deg:.8f}</longitude_deg>
      <elevation>{course.datum_altitude_m:.3f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    {"".join(models)}
  </world>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--course-config", default=str(DEFAULT_COURSE_CONFIG))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    course = load_course(args.course_config)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(generate_world(course), encoding="utf-8")


if __name__ == "__main__":
    main()
