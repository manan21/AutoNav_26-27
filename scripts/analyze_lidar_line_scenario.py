#!/usr/bin/env python3
"""Analyze a lidar-line ROS course bag against scenario metadata."""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/local_ekf/odom",
    "/navigate_to_pose/_action/status",
}

TERMINAL = {4, 5, 6}


@dataclass(frozen=True)
class Tape:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]
    width_m: float


@dataclass(frozen=True)
class Cone:
    name: str
    center: tuple[float, float]
    radius_m: float


@dataclass(frozen=True)
class Station:
    label: str
    x_m: float
    y_min_m: float
    y_max_m: float


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    description: str
    tapes: tuple[Tape, ...]
    cones: tuple[Cone, ...]
    stations: tuple[Station, ...]


def parse_scalar(raw: str) -> object:
    value = raw.split("#", 1)[0].strip()
    if not value:
        return ""
    if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_flat_config(path: Path) -> dict[str, object]:
    values: dict[str, object] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, raw = stripped.split(":", 1)
        values[key.strip()] = parse_scalar(raw)
    return values


def cfg_float(values: dict[str, object], key: str,
              default: float | None = None) -> float:
    if key not in values:
        if default is None:
            raise KeyError(key)
        return float(default)
    return float(values[key])


def cfg_int(values: dict[str, object], key: str, default: int = 0) -> int:
    return int(values.get(key, default))


def cfg_str(values: dict[str, object], key: str, default: str = "") -> str:
    return str(values.get(key, default))


def load_scenario(path: Path) -> Scenario:
    values = load_flat_config(path)
    if "tape_count" in values:
        tapes = load_indexed_tapes(values)
        cones = load_indexed_cones(values)
        stations = load_indexed_stations(values)
        return Scenario(
            scenario_id=cfg_str(values, "scenario_id", path.stem),
            description=cfg_str(values, "description", path.stem),
            tapes=tapes,
            cones=cones,
            stations=stations,
        )
    return load_legacy_scenario(values)


def load_indexed_tapes(values: dict[str, object]) -> tuple[Tape, ...]:
    default_width = cfg_float(values, "tape_width_m", 0.12)
    tapes: list[Tape] = []
    for idx in range(cfg_int(values, "tape_count")):
        prefix = f"tape_{idx}_"
        tapes.append(Tape(
            name=cfg_str(values, f"{prefix}name", f"tape_{idx}"),
            start=(
                cfg_float(values, f"{prefix}start_x_m"),
                cfg_float(values, f"{prefix}start_y_m"),
            ),
            end=(
                cfg_float(values, f"{prefix}end_x_m"),
                cfg_float(values, f"{prefix}end_y_m"),
            ),
            width_m=cfg_float(values, f"{prefix}width_m", default_width),
        ))
    return tuple(tapes)


def load_indexed_cones(values: dict[str, object]) -> tuple[Cone, ...]:
    default_radius = cfg_float(values, "cone_radius_m", 0.23)
    cones: list[Cone] = []
    for idx in range(cfg_int(values, "cone_count")):
        prefix = f"cone_{idx}_"
        cones.append(Cone(
            name=cfg_str(values, f"{prefix}name", f"cone_{idx}"),
            center=(
                cfg_float(values, f"{prefix}center_x_m"),
                cfg_float(values, f"{prefix}center_y_m"),
            ),
            radius_m=cfg_float(values, f"{prefix}radius_m", default_radius),
        ))
    return tuple(cones)


def load_indexed_stations(values: dict[str, object]) -> tuple[Station, ...]:
    stations: list[Station] = []
    for idx in range(cfg_int(values, "analysis_station_count")):
        prefix = f"analysis_station_{idx}_"
        stations.append(Station(
            label=cfg_str(values, f"{prefix}label", f"station_{idx}"),
            x_m=cfg_float(values, f"{prefix}x_m"),
            y_min_m=cfg_float(values, f"{prefix}y_min_m"),
            y_max_m=cfg_float(values, f"{prefix}y_max_m"),
        ))
    return tuple(stations)


def load_legacy_scenario(values: dict[str, object]) -> Scenario:
    lidar_x_from_nav = cfg_float(values, "lidar_x_from_nav_center_m")
    tape_width = cfg_float(values, "tape_width_m")
    left_y = cfg_float(values, "left_tape_y_m")
    left_start = cfg_float(values, "left_tape_lidar_start_x_m") + lidar_x_from_nav
    left_end = left_start + cfg_float(values, "left_tape_length_m")
    perp_x = cfg_float(values, "perpendicular_tape_lidar_x_m") + lidar_x_from_nav
    perp_left_y = cfg_float(values, "perpendicular_tape_left_y_m")
    perp_right_y = cfg_float(values, "perpendicular_tape_right_y_m")
    cone_radius = cfg_float(values, "cone_radius_m")
    cone_left = cfg_float(values, "cone_left_boundary_y_m")
    return Scenario(
        scenario_id="canonical_5ft_gap",
        description="Legacy canonical lidar-line course",
        tapes=(
            Tape("left_tape", (left_start, left_y), (left_end, left_y), tape_width),
            Tape("perpendicular_tape", (perp_x, perp_left_y),
                 (perp_x, perp_right_y), tape_width),
        ),
        cones=(
            Cone("dot_cone", (perp_x, cone_left - cone_radius), cone_radius),
        ),
        stations=(
            Station(
                "through_5ft_gap",
                perp_x,
                cfg_float(values, "required_centerline_cone_side_y_m"),
                cfg_float(values, "required_centerline_tape_side_y_m"),
            ),
        ),
    )


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_from_odom(msg, nav_center_x: float) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    return (
        p.x + nav_center_x * math.cos(yaw),
        p.y + nav_center_x * math.sin(yaw),
        yaw,
    )


def nearest_index(times: list[float], stamp: float) -> int:
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    return idx - 1 if abs(times[idx - 1] - stamp) <= abs(times[idx] - stamp) else idx


def signed_box_clearance(x: float, y: float, half_x: float, half_y: float) -> float:
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


def point_clearance_to_pose(point: tuple[float, float],
                            pose: tuple[float, float, float],
                            half_length: float,
                            half_width: float) -> float:
    rx, ry, ryaw = pose
    px, py = point
    dx = px - rx
    dy = py - ry
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    robot_x = c * dx + s * dy
    robot_y = -s * dx + c * dy
    return signed_box_clearance(robot_x, robot_y, half_length, half_width)


def circle_clearance_to_pose(cone: Cone,
                             pose: tuple[float, float, float],
                             half_length: float,
                             half_width: float) -> float:
    center_clearance = point_clearance_to_pose(
        cone.center,
        pose,
        half_length,
        half_width,
    )
    return center_clearance - cone.radius_m


def sample_tape(tape: Tape, step: float) -> list[tuple[float, float]]:
    ax, ay = tape.start
    bx, by = tape.end
    length = math.hypot(bx - ax, by - ay)
    count = max(1, int(math.ceil(length / max(step, 1e-3))))
    points = []
    for idx in range(count + 1):
        t = idx / count
        points.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return points


def y_at_x(points: list[tuple[float, float]], target_x: float) -> float | None:
    for a, b in zip(points, points[1:]):
        ax, ay = a
        bx, by = b
        if (ax <= target_x <= bx) or (bx <= target_x <= ax):
            if abs(bx - ax) < 1e-6:
                return 0.5 * (ay + by)
            t = (target_x - ax) / (bx - ax)
            return ay + t * (by - ay)
    return None


def bag_messages(bag_path: str, topics: set[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected = {
        topic: get_message(topic_types[topic])
        for topic in topics
        if topic in topic_types
    }
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg_type = selected.get(topic)
        if msg_type is None:
            continue
        yield topic, deserialize_message(data, msg_type), stamp_ns * 1e-9


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--scenario-config", required=True)
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--half-length", type=float, default=0.595)
    parser.add_argument("--half-width", type=float, default=0.46)
    parser.add_argument("--padding", type=float, default=0.05)
    parser.add_argument("--sample-step", type=float, default=0.02)
    parser.add_argument("--fail-on-overlap", action="store_true")
    parser.add_argument("--fail-on-padded-overlap", action="store_true")
    parser.add_argument("--strict-stations", action="store_true")
    args = parser.parse_args()

    scenario = load_scenario(Path(args.scenario_config).expanduser())
    odom = []
    nav_events = []
    bag_start = None
    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        if topic == "/local_ekf/odom":
            odom.append((stamp, pose_from_odom(msg, args.nav_center_x)))
        elif topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                nav_events.append((stamp, int(status.status)))

    if bag_start is None:
        raise SystemExit("empty bag")
    if not odom:
        raise SystemExit("no /local_ekf/odom samples")

    exec_times = [stamp for stamp, state in nav_events if state == 2]
    terminal_times = [stamp for stamp, state in nav_events if state in TERMINAL]
    action_start = min(exec_times) if exec_times else odom[0][0]
    action_end = min(
        (stamp for stamp in terminal_times if stamp >= action_start),
        default=odom[-1][0],
    )
    odom_times = [stamp for stamp, _ in odom]
    start_idx = nearest_index(odom_times, action_start)
    start_pose = odom[start_idx][1]

    samples = [
        (stamp, pose)
        for stamp, pose in odom
        if action_start <= stamp <= action_end
    ]
    if not samples:
        raise SystemExit("no odom samples in NavigateToPose execution window")

    physical_half = (args.half_length, args.half_width)
    padded_half = (
        args.half_length + args.padding,
        args.half_width + args.padding,
    )
    tape_points = [
        (tape, point)
        for tape in scenario.tapes
        for point in sample_tape(tape, args.sample_step)
    ]

    best_tape_physical = None
    best_tape_padded = None
    best_cone_physical = None
    best_cone_padded = None
    tape_physical_overlaps = 0
    tape_padded_overlaps = 0
    cone_physical_overlaps = 0
    cone_padded_overlaps = 0

    for stamp, pose in samples:
        for tape, point in tape_points:
            tape_radius = 0.5 * tape.width_m
            physical = point_clearance_to_pose(point, pose, *physical_half) - tape_radius
            padded = point_clearance_to_pose(point, pose, *padded_half) - tape_radius
            physical_sample = (physical, stamp, pose, tape.name, point)
            padded_sample = (padded, stamp, pose, tape.name, point)
            if best_tape_physical is None or physical < best_tape_physical[0]:
                best_tape_physical = physical_sample
            if best_tape_padded is None or padded < best_tape_padded[0]:
                best_tape_padded = padded_sample
            if physical <= 0.0:
                tape_physical_overlaps += 1
            if padded <= 0.0:
                tape_padded_overlaps += 1

        for cone in scenario.cones:
            physical = circle_clearance_to_pose(cone, pose, *physical_half)
            padded = circle_clearance_to_pose(cone, pose, *padded_half)
            physical_sample = (physical, stamp, pose, cone.name, cone.center)
            padded_sample = (padded, stamp, pose, cone.name, cone.center)
            if best_cone_physical is None or physical < best_cone_physical[0]:
                best_cone_physical = physical_sample
            if best_cone_padded is None or padded < best_cone_padded[0]:
                best_cone_padded = padded_sample
            if physical <= 0.0:
                cone_physical_overlaps += 1
            if padded <= 0.0:
                cone_padded_overlaps += 1

    traj = [(pose[0], pose[1]) for _, pose in samples]
    xs = [x for x, _ in traj]
    ys = [y for _, y in traj]
    final_pose = samples[-1][1]

    print("Lidar-line scenario analysis")
    print(f"bag: {args.bag}")
    print(f"scenario: {scenario.scenario_id} - {scenario.description}")
    print(
        f"window: {action_start - bag_start:.2f}s..{action_end - bag_start:.2f}s "
        f"samples={len(samples)}"
    )
    print(
        f"start=({start_pose[0]:+.3f},{start_pose[1]:+.3f},"
        f"{math.degrees(start_pose[2]):+.1f}deg) "
        f"final=({final_pose[0]:+.3f},{final_pose[1]:+.3f},"
        f"{math.degrees(final_pose[2]):+.1f}deg)"
    )
    print(
        f"trajectory x=[{min(xs):+.3f},{max(xs):+.3f}] "
        f"y=[{min(ys):+.3f},{max(ys):+.3f}]"
    )

    failures = []

    def print_best(label: str, best, overlaps: int) -> None:
        if best is None:
            print(f"{label}: none")
            return
        clearance, stamp, pose, name, point = best
        print(
            f"{label}: clearance={clearance:+.3f}m overlaps={overlaps} "
            f"at t={stamp - action_start:.2f}s pose=({pose[0]:+.3f},"
            f"{pose[1]:+.3f},{math.degrees(pose[2]):+.1f}deg) "
            f"object={name} point=({point[0]:+.3f},{point[1]:+.3f})"
        )

    print_best("physical tape", best_tape_physical, tape_physical_overlaps)
    print_best("padded tape", best_tape_padded, tape_padded_overlaps)
    print_best("physical cone", best_cone_physical, cone_physical_overlaps)
    print_best("padded cone", best_cone_padded, cone_padded_overlaps)

    if args.fail_on_overlap:
        if tape_physical_overlaps:
            failures.append(
                f"physical footprint overlapped tape in {tape_physical_overlaps} samples")
        if cone_physical_overlaps:
            failures.append(
                f"physical footprint overlapped cone in {cone_physical_overlaps} samples")

    if args.fail_on_padded_overlap:
        if tape_padded_overlaps:
            failures.append(
                f"padded footprint overlapped tape in {tape_padded_overlaps} samples")
        if cone_padded_overlaps:
            failures.append(
                f"padded footprint overlapped cone in {cone_padded_overlaps} samples")

    print("\nscenario stations")
    if not scenario.stations:
        print("  none")
    for station in scenario.stations:
        y_value = y_at_x(traj, station.x_m)
        if y_value is None:
            verdict = "missing"
            ok = False
        else:
            ok = station.y_min_m <= y_value <= station.y_max_m
            verdict = "ok" if ok else "outside"
        y_text = "none" if y_value is None else f"{y_value:+.3f}"
        print(
            f"  {station.label}: x={station.x_m:+.3f} y={y_text} "
            f"expected=[{station.y_min_m:+.3f},{station.y_max_m:+.3f}] "
            f"{verdict}"
        )
        if args.strict_stations and not ok:
            failures.append(f"station {station.label} outside expected band")

    if failures:
        print("\nFAIL: lidar-line scenario analysis")
        for failure in failures:
            print(f"  {failure}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
