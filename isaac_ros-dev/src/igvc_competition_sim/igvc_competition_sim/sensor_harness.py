#!/usr/bin/env python3
from __future__ import annotations

import math
import random
import sys

from .course import Course, load_course, local_to_latlon
from .lidar_geometry import raycast_cylinders

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from builtin_interfaces.msg import Time
    from autonav_interfaces.msg import LinePoints
    from geometry_msgs.msg import TransformStamped, Twist, Vector3
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import (
        JointState,
        LaserScan,
        NavSatFix,
        NavSatStatus,
        PointCloud2,
        PointField,
    )
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Bool, Header
    from tf2_ros import TransformBroadcaster
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_sensor_harness must run in a sourced ROS 2 Humble environment"
    ) from exc

# RCLError was added to rclpy after Humble; fall back when absent (matches the
# pattern already in dynamics_replay.py). auto_camera env-compat fix.
try:
    from rclpy.exceptions import RCLError
except ImportError:
    RCLError = Exception


MULTISCAN_LAYERS = 16
LIDAR_AZIMUTH_MIN_RAD = -math.pi / 2.0
LIDAR_AZIMUTH_MAX_RAD = math.pi / 2.0
LIDAR_ELEVATION_MIN_RAD = math.radians(-35.0)
LIDAR_ELEVATION_MAX_RAD = math.radians(7.5)
LIDAR_HORIZONTAL_RES_RAD = math.radians(0.5)
RANGE_MIN_M = 0.20
RANGE_MAX_M = 8.5
FLOOR_RSSI = 30000.0
TAPE_RSSI = 52000.0
POTHOLE_RSSI = 50000.0
OBSTACLE_RSSI = 33000.0
OBSTACLE_REFLECTOR_RSSI = 50000.0
CMD_TIMEOUT_S = 0.40


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _stamp_to_float(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class IgvcSensorHarness(Node):
    def __init__(self) -> None:
        super().__init__("igvc_sensor_harness")
        self.declare_parameter("course_config", "")
        self.declare_parameter("fallback_integrate_cmd", False)
        self.declare_parameter("publish_ground_truth_pca", False)
        self.declare_parameter("publish_ground_truth_lines", False)
        self.declare_parameter("cloud_rate_hz", 10.0)
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("map_rate_hz", 1.0)
        self.declare_parameter("gps_rate_hz", 10.0)
        self.declare_parameter("ground_truth_line_rate_hz", 10.0)
        self.declare_parameter("ground_truth_line_spacing_m", 0.05)
        self.declare_parameter("ground_truth_line_lateral_spacing_m", 0.025)
        self.declare_parameter("gps_noise_std_m", 0.05)
        self.declare_parameter("publish_ground_truth_odom", True)
        self.declare_parameter(
            "ground_truth_odom_topic", "/igvc_sim/ground_truth_odom")

        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        self.robot = self.course.robot
        self.fallback_integrate_cmd = bool(
            self.get_parameter("fallback_integrate_cmd").value)
        self.publish_ground_truth_pca = bool(
            self.get_parameter("publish_ground_truth_pca").value)
        self.publish_ground_truth_lines = bool(
            self.get_parameter("publish_ground_truth_lines").value)
        self.gps_noise_std_m = max(
            0.0, float(self.get_parameter("gps_noise_std_m").value))

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        line_qos = QoSProfile(depth=1)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cloud_pub = self.create_publisher(
            PointCloud2, "/cloud_all_fields_fullframe", sensor_qos)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_fullframe", sensor_qos)
        self.pca_gt_pub = (
            self.create_publisher(
                PointCloud2, "/scan_pca_filtered_points", sensor_qos)
            if self.publish_ground_truth_pca else None
        )
        self.line_gt_pub = (
            self.create_publisher(LinePoints, "/line_points", line_qos)
            if self.publish_ground_truth_lines else None
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/map_padded", map_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.local_odom_pub = self.create_publisher(
            Odometry, "/local_ekf/odom", 10)
        self.ground_truth_odom_pub = (
            self.create_publisher(
                Odometry,
                str(self.get_parameter("ground_truth_odom_topic").value),
                10,
            )
            if bool(self.get_parameter("publish_ground_truth_odom").value)
            else None
        )
        self.gps_pub = self.create_publisher(NavSatFix, "/gps_fix", sensor_qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.autonomous_pub = self.create_publisher(
            Bool, "/autonomous_mode", 1)

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.gz_odom_sub = self.create_subscription(
            Odometry, "/model/shogi/odometry", self._gazebo_odom_callback, 10)

        self.tf_pub = TransformBroadcaster(self)
        self.base_x = self.course.start.x
        self.base_y = self.course.start.y
        self.heading = self.course.start.yaw
        self.applied_v = 0.0
        self.applied_w = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.pending_commands: list[tuple[float, float, float]] = []
        self.last_cmd_s = -math.inf
        self.last_gazebo_odom_s = -math.inf
        self.last_step_s: float | None = None
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.ground_truth_line_points = self._build_ground_truth_line_points()

        self.create_timer(
            1.0 / max(1.0, float(self.get_parameter("odom_rate_hz").value)),
            self._step_and_publish_odom,
        )
        self.create_timer(
            1.0 / max(1.0, float(self.get_parameter("cloud_rate_hz").value)),
            self._publish_sensor_frame,
        )
        self.create_timer(
            1.0 / max(0.5, float(self.get_parameter("map_rate_hz").value)),
            self._publish_map,
        )
        self.create_timer(
            1.0 / max(0.5, float(self.get_parameter("gps_rate_hz").value)),
            self._publish_gps,
        )
        if self.line_gt_pub is not None:
            self.create_timer(
                1.0 / max(
                    0.5,
                    float(self.get_parameter("ground_truth_line_rate_hz").value),
                ),
                self._publish_ground_truth_lines,
            )
        self._publish_map()
        self.get_logger().info(
            "IGVC competition harness loaded %s: tapes=%d obstacles=%d "
            "potholes=%d ramps=%d mission_waypoints=%d"
            % (
                self.course.course_id,
                len(self.course.tapes),
                len(self.course.obstacles),
                len(self.course.potholes),
                len(self.course.ramps),
                len(self.course.mission_waypoints),
            )
        )
        if self.line_gt_pub is not None:
            self.get_logger().info(
                "Publishing ground-truth /line_points from %d sampled tape cells"
                % len(self.ground_truth_line_points)
            )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        now_s = _stamp_to_float(self.get_clock().now().to_msg())
        self.pending_commands.append((
            now_s + self.robot.cmd_latency_s,
            _clamp(float(msg.linear.x),
                   -self.robot.max_linear_speed_mps,
                   self.robot.max_linear_speed_mps),
            _clamp(float(msg.angular.z),
                   -self.robot.max_angular_speed_radps,
                   self.robot.max_angular_speed_radps),
        ))
        self.last_cmd_s = now_s

    def _gazebo_odom_callback(self, msg: Odometry) -> None:
        self.last_gazebo_odom_s = _stamp_to_float(
            self.get_clock().now().to_msg())
        self.base_x = float(msg.pose.pose.position.x)
        self.base_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.heading = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.applied_v = float(msg.twist.twist.linear.x)
        self.applied_w = float(msg.twist.twist.angular.z)

    def _step_and_publish_odom(self) -> None:
        stamp = self.get_clock().now().to_msg()
        now_s = _stamp_to_float(stamp)
        if self.last_step_s is None:
            self.last_step_s = now_s
        dt = max(0.0, min(0.10, now_s - self.last_step_s))
        self.last_step_s = now_s

        if self.fallback_integrate_cmd and now_s - self.last_gazebo_odom_s > 1.0:
            self._integrate_cmd_fallback(now_s, dt)
        self._integrate_wheels(dt, self.applied_v, self.applied_w)
        self._publish_dynamic_transforms(stamp)
        self._publish_odom(stamp)
        self._publish_joint_states(stamp)
        self.autonomous_pub.publish(Bool(data=True))

    def _integrate_cmd_fallback(self, now_s: float, dt: float) -> None:
        while self.pending_commands and self.pending_commands[0][0] <= now_s:
            _, self.cmd_v, self.cmd_w = self.pending_commands.pop(0)
        if now_s - self.last_cmd_s > CMD_TIMEOUT_S:
            target_v = 0.0
            target_w = 0.0
        else:
            target_v = self.cmd_v
            target_w = self.cmd_w
        self.applied_v = self._first_order(
            self.applied_v, target_v, dt, self.robot.linear_time_constant_s)
        self.applied_w = self._first_order(
            self.applied_w, target_w, dt, self.robot.angular_time_constant_s)
        self.base_x += self.applied_v * math.cos(self.heading) * dt
        self.base_y += self.applied_v * math.sin(self.heading) * dt
        self.heading = math.atan2(
            math.sin(self.heading + self.applied_w * dt),
            math.cos(self.heading + self.applied_w * dt),
        )

    @staticmethod
    def _first_order(current: float, target: float, dt: float, tau: float
                     ) -> float:
        if tau <= 1e-6:
            return target
        alpha = 1.0 - math.exp(-max(0.0, dt) / tau)
        return current + alpha * (target - current)

    def _integrate_wheels(self, dt: float, v: float, w: float) -> None:
        left_linear = v - w * self.robot.wheel_track_m * 0.5
        right_linear = v + w * self.robot.wheel_track_m * 0.5
        self.left_wheel_position += left_linear / self.robot.wheel_radius_m * dt
        self.right_wheel_position += right_linear / self.robot.wheel_radius_m * dt

    def _publish_joint_states(self, stamp: Time) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ["Left_Wheel", "Right_Wheel"]
        msg.position = [self.left_wheel_position, self.right_wheel_position]
        msg.velocity = [
            (self.applied_v - self.applied_w * self.robot.wheel_track_m * 0.5)
            / self.robot.wheel_radius_m,
            (self.applied_v + self.applied_w * self.robot.wheel_track_m * 0.5)
            / self.robot.wheel_radius_m,
        ]
        self.joint_pub.publish(msg)

    def _publish_dynamic_transforms(self, stamp: Time) -> None:
        transforms: list[TransformStamped] = []
        transforms.append(self._transform(
            stamp, "map", "odom", 0.0, 0.0, 0.0, *_yaw_quaternion(0.0)))
        transforms.append(self._transform(
            stamp, "odom", "base_link", self.base_x, self.base_y, 0.0,
            *_yaw_quaternion(self.heading)))
        self.tf_pub.sendTransform(transforms)

    @staticmethod
    def _transform(stamp: Time,
                   parent: str,
                   child: str,
                   x: float,
                   y: float,
                   z: float,
                   qx: float,
                   qy: float,
                   qz: float,
                   qw: float) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = float(z)
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        return msg

    def _publish_odom(self, stamp: Time) -> None:
        qx, qy, qz, qw = _yaw_quaternion(self.heading)
        for publisher in (self.odom_pub, self.local_odom_pub):
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = self.base_x
            msg.pose.pose.position.y = self.base_y
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = self.applied_v
            msg.twist.twist.angular.z = self.applied_w
            publisher.publish(msg)
        if self.ground_truth_odom_pub is not None:
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = self.base_x
            msg.pose.pose.position.y = self.base_y
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = self.applied_v
            msg.twist.twist.angular.z = self.applied_w
            self.ground_truth_odom_pub.publish(msg)

    def _publish_gps(self) -> None:
        stamp = self.get_clock().now().to_msg()
        c = math.cos(self.heading)
        s = math.sin(self.heading)
        gps_x = (
            self.base_x + self.robot.gps_x_from_base_link_m * c
            - self.robot.gps_y_from_base_link_m * s
        )
        gps_y = (
            self.base_y + self.robot.gps_x_from_base_link_m * s
            + self.robot.gps_y_from_base_link_m * c
        )
        if self.gps_noise_std_m > 0.0:
            gps_x += random.gauss(0.0, self.gps_noise_std_m)
            gps_y += random.gauss(0.0, self.gps_noise_std_m)
        lat, lon = local_to_latlon(
            gps_x,
            gps_y,
            self.course.datum_latitude_deg,
            self.course.datum_longitude_deg,
        )
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = "gps_footprint"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = self.course.datum_altitude_m
        variance = max(0.01, self.gps_noise_std_m ** 2)
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, max(1.0, variance),
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(msg)

    def _lidar_origin_world(self) -> tuple[float, float]:
        c = math.cos(self.heading)
        s = math.sin(self.heading)
        return (
            self.base_x + self.robot.lidar_x_from_base_link_m * c,
            self.base_y + self.robot.lidar_x_from_base_link_m * s,
        )

    def _world_to_base(self,
                       x: float,
                       y: float,
                       z_above_ground: float = 0.0,
                       ) -> tuple[float, float, float]:
        dx = x - self.base_x
        dy = y - self.base_y
        c = math.cos(self.heading)
        s = math.sin(self.heading)
        return (
            c * dx + s * dy,
            -s * dx + c * dy,
            -self.robot.base_link_height_above_ground_m + z_above_ground,
        )

    def _base_to_lidar(self, base: tuple[float, float, float]
                       ) -> tuple[float, float, float]:
        bx, by, bz = base
        return (
            bx - self.robot.lidar_x_from_base_link_m,
            -by,
            -(bz - self.robot.lidar_z_from_base_link_m),
        )

    @staticmethod
    def _cloud_tuple(lidar: tuple[float, float, float],
                     rng: float,
                     intensity: float,
                     reflector: bool,
                     layer: int) -> tuple[float, float, float, float,
                                          float, float, float, float]:
        return (
            float(lidar[0]),
            float(lidar[1]),
            float(lidar[2]),
            float(intensity),
            float(rng),
            float(layer),
            0.0,
            1.0 if reflector else 0.0,
        )

    def _append_cloud_point(
        self,
        out: list[tuple[float, float, float, float,
                        float, float, float, float]],
        world_x: float,
        world_y: float,
        z_above_ground: float,
        intensity: float,
        reflector: bool,
        layer: int,
    ) -> None:
        lidar = self._base_to_lidar(
            self._world_to_base(world_x, world_y, z_above_ground))
        rng = math.sqrt(lidar[0] ** 2 + lidar[1] ** 2 + lidar[2] ** 2)
        if RANGE_MIN_M <= rng <= RANGE_MAX_M:
            out.append(self._cloud_tuple(
                lidar, rng, intensity, reflector, layer))

    def _raycast_obstacle_hits(self):
        sensor_height = (
            self.robot.base_link_height_above_ground_m
            + self.robot.lidar_z_from_base_link_m
        )
        return raycast_cylinders(
            self.course.obstacles,
            self._lidar_origin_world(),
            self.heading,
            sensor_height,
            LIDAR_AZIMUTH_MIN_RAD,
            LIDAR_AZIMUTH_MAX_RAD,
            LIDAR_HORIZONTAL_RES_RAD,
            LIDAR_ELEVATION_MIN_RAD,
            LIDAR_ELEVATION_MAX_RAD,
            MULTISCAN_LAYERS,
            RANGE_MIN_M,
            RANGE_MAX_M,
        )

    def _build_cloud_points(self) -> list[tuple[float, float, float, float,
                                               float, float, float, float]]:
        points: list[tuple[float, float, float, float,
                           float, float, float, float]] = []
        obstacle_hits = {
            (hit.layer, round(hit.azimuth_rad, 9)): hit
            for hit in self._raycast_obstacle_hits()
        }
        azimuth_count = int(math.floor(
            (LIDAR_AZIMUTH_MAX_RAD - LIDAR_AZIMUTH_MIN_RAD)
            / LIDAR_HORIZONTAL_RES_RAD)) + 1
        origin_x, origin_y = self._lidar_origin_world()
        sensor_height = (
            self.robot.base_link_height_above_ground_m
            + self.robot.lidar_z_from_base_link_m
        )
        for layer in range(MULTISCAN_LAYERS):
            if MULTISCAN_LAYERS == 1:
                elevation = 0.5 * (LIDAR_ELEVATION_MIN_RAD
                                   + LIDAR_ELEVATION_MAX_RAD)
            else:
                frac = layer / float(MULTISCAN_LAYERS - 1)
                elevation = LIDAR_ELEVATION_MIN_RAD + (
                    LIDAR_ELEVATION_MAX_RAD - LIDAR_ELEVATION_MIN_RAD) * frac
            cos_elevation = math.cos(elevation)
            tan_elevation = math.tan(elevation)
            if cos_elevation <= 1e-6:
                continue
            for azimuth_idx in range(azimuth_count):
                azimuth = LIDAR_AZIMUTH_MIN_RAD + (
                    LIDAR_HORIZONTAL_RES_RAD * azimuth_idx)
                best_range = math.inf
                best: tuple[float, float, float, float, bool] | None = None
                hit = obstacle_hits.get((layer, round(azimuth, 9)))
                if hit is not None:
                    reflector = hit.z > 0.35
                    best_range = hit.range_m
                    best = (
                        hit.x,
                        hit.y,
                        hit.z,
                        OBSTACLE_REFLECTOR_RSSI
                        if reflector else OBSTACLE_RSSI,
                        reflector,
                    )
                if tan_elevation < -1e-6:
                    horizontal_distance = -sensor_height / tan_elevation
                    floor_range = horizontal_distance / cos_elevation
                    if RANGE_MIN_M <= floor_range <= RANGE_MAX_M:
                        world_angle = self.heading + azimuth
                        floor_x = origin_x + horizontal_distance * math.cos(
                            world_angle)
                        floor_y = origin_y + horizontal_distance * math.sin(
                            world_angle)
                        if floor_range < best_range:
                            on_tape = self._point_on_tape(floor_x, floor_y)
                            on_pothole = self._point_in_pothole(
                                floor_x, floor_y)
                            best_range = floor_range
                            best = (
                                floor_x,
                                floor_y,
                                0.0,
                                POTHOLE_RSSI if on_pothole else (
                                    TAPE_RSSI if on_tape else FLOOR_RSSI),
                                on_tape or on_pothole,
                            )
                if best is None:
                    continue
                world_x, world_y, z_above_ground, intensity, reflector = best
                self._append_cloud_point(
                    points,
                    world_x,
                    world_y,
                    z_above_ground,
                    intensity,
                    reflector,
                    layer,
                )
        return points

    def _ground_truth_obstacle_points(self) -> list[tuple[float, float, float,
                                                         float, float, float,
                                                         float, float]]:
        points: list[tuple[float, float, float, float,
                           float, float, float, float]] = []
        for hit in self._raycast_obstacle_hits():
            self._append_cloud_point(
                points, hit.x, hit.y, hit.z, OBSTACLE_RSSI, False, hit.layer)
        return points

    def _point_on_tape(self, x: float, y: float) -> bool:
        for tape in self.course.tapes:
            if self._point_segment_distance(x, y, tape.start, tape.end) <= (
                    0.5 * tape.width_m):
                return True
        return False

    def _point_in_pothole(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - pothole.center[0], y - pothole.center[1])
            <= pothole.radius_m
            for pothole in self.course.potholes
        )

    @staticmethod
    def _point_segment_distance(x: float,
                                y: float,
                                start: tuple[float, float],
                                end: tuple[float, float]) -> float:
        ax, ay = start
        bx, by = end
        abx = bx - ax
        aby = by - ay
        denom = abx * abx + aby * aby
        if denom <= 1e-12:
            return math.hypot(x - ax, y - ay)
        t = max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / denom))
        qx = ax + t * abx
        qy = ay + t * aby
        return math.hypot(x - qx, y - qy)

    def _make_cloud(self,
                    stamp: Time,
                    points: list[tuple[float, float, float, float,
                                       float, float, float, float]],
                    frame_id: str = "lidar_footprint") -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="i", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="range", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="layer", offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name="echo", offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name="reflector", offset=28, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return point_cloud2.create_cloud(
            header=header, fields=fields, points=points)

    def _publish_sensor_frame(self) -> None:
        stamp = self.get_clock().now().to_msg()
        cloud_points = self._build_cloud_points()
        self.cloud_pub.publish(self._make_cloud(stamp, cloud_points))
        self._publish_scan_fullframe(stamp, cloud_points)
        if self.pca_gt_pub is not None:
            self.pca_gt_pub.publish(
                self._make_cloud(stamp, self._ground_truth_obstacle_points()))

    def _publish_scan_fullframe(self,
                                stamp: Time,
                                points: list[tuple[float, float, float, float,
                                                   float, float, float, float]]
                                ) -> None:
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "lidar_footprint"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(0.5)
        scan.scan_time = 0.1
        scan.range_min = 0.20
        scan.range_max = 25.0
        count = int(round((scan.angle_max - scan.angle_min)
                          / scan.angle_increment)) + 1
        ranges = [math.inf] * count
        for x, y, _z, _i, rng, _layer, _echo, _reflector in points:
            angle = math.atan2(y, x)
            idx = int(round((angle - scan.angle_min) / scan.angle_increment))
            if 0 <= idx < count and rng < ranges[idx]:
                ranges[idx] = float(rng)
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def _build_ground_truth_line_points(self) -> list[Vector3]:
        spacing = max(
            0.01, float(self.get_parameter("ground_truth_line_spacing_m").value))
        lateral_spacing = max(
            0.01,
            float(self.get_parameter("ground_truth_line_lateral_spacing_m").value),
        )
        points: list[Vector3] = []
        for tape in self.course.tapes:
            ax, ay = tape.start
            bx, by = tape.end
            dx = bx - ax
            dy = by - ay
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            ux = dx / length
            uy = dy / length
            nx = -uy
            ny = ux
            longitudinal_steps = max(1, int(math.ceil(length / spacing)))
            lateral_steps = max(1, int(math.ceil(tape.width_m / lateral_spacing)))
            for i in range(longitudinal_steps + 1):
                along = min(length, i * spacing)
                cx = ax + ux * along
                cy = ay + uy * along
                for j in range(lateral_steps + 1):
                    if lateral_steps == 1:
                        offset = 0.0
                    else:
                        offset = (
                            -0.5 * tape.width_m
                            + tape.width_m * j / float(lateral_steps)
                        )
                    point = Vector3()
                    point.x = cx + nx * offset
                    point.y = cy + ny * offset
                    point.z = 0.0
                    points.append(point)
        return points

    def _publish_ground_truth_lines(self) -> None:
        if self.line_gt_pub is None:
            return
        msg = LinePoints()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.points = self.ground_truth_line_points
        self.line_gt_pub.publish(msg)

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.05
        min_x, min_y, max_x, max_y = self._map_bounds(msg.info.resolution)
        msg.info.width = int(math.ceil((max_x - min_x) / msg.info.resolution))
        msg.info.height = int(math.ceil((max_y - min_y) / msg.info.resolution))
        msg.info.origin.position.x = min_x
        msg.info.origin.position.y = min_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (msg.info.width * msg.info.height)
        self.map_pub.publish(msg)

    def _map_bounds(self, resolution: float) -> tuple[float, float, float, float]:
        from .course import course_bounds
        min_x, min_y, max_x, max_y = course_bounds(self.course, margin_m=6.0)
        return (
            math.floor(min_x / resolution) * resolution,
            math.floor(min_y / resolution) * resolution,
            math.ceil(max_x / resolution) * resolution,
            math.ceil(max_y / resolution) * resolution,
        )


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=sys.argv if argv is None else argv)
    node = IgvcSensorHarness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
