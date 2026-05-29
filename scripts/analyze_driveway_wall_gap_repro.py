#!/usr/bin/env python3
"""Diagnose driveway wall/line gap plans and mirror-layer interactions.

This analyzer is intentionally geometry-specific: it expects a driveway
scenario config with a horizontal right wall and a diagonal trap tape. It
answers whether /plan enters the wall-side gap and whether local wall cells
are missing from the raw global costmap where /lidar_line_costmap has halo.
"""

from __future__ import annotations

import argparse
import bisect
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/local_ekf/odom",
    "/navigate_to_pose/_action/status",
    "/plan",
    "/lidar_line_costmap",
    "/local_costmap/costmap",
    "/global_costmap/costmap_raw",
}
TERMINAL = {4, 5, 6}


def load_flat_config(path: Path) -> dict[str, object]:
    values: dict[str, object] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, raw_value = stripped.split(":", 1)
        raw = raw_value.split("#", 1)[0].strip().strip("\"'")
        if not raw:
            values[key.strip()] = ""
            continue
        lowered = raw.lower()
        if lowered in ("true", "false"):
            values[key.strip()] = lowered == "true"
            continue
        try:
            values[key.strip()] = float(raw) if any(
                ch in raw for ch in ".eE") else int(raw)
        except ValueError:
            values[key.strip()] = raw
    return values


def cfg_float(values: dict[str, object], key: str) -> float:
    return float(values[key])


def cfg_str(values: dict[str, object], key: str, default: str) -> str:
    return str(values.get(key, default))


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


def nearest_pose(times: list[float],
                 poses: list[tuple[float, float, float]],
                 stamp: float) -> tuple[float, float, float] | None:
    if not times:
        return None
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return poses[0]
    if idx >= len(times):
        return poses[-1]
    return poses[idx - 1] if abs(times[idx - 1] - stamp) <= abs(
        times[idx] - stamp) else poses[idx]


def preceding_entry(entries, stamp):
    if not entries:
        return None
    times = [entry[0] for entry in entries]
    idx = bisect.bisect_right(times, stamp) - 1
    if idx < 0:
        return None
    return entries[idx]


def entries_in_window(entries, times: list[float], start: float, end: float):
    first = bisect.bisect_left(times, start)
    last = bisect.bisect_right(times, end)
    return entries[first:last]


def path_points(msg,
                pose: tuple[float, float, float] | None
                ) -> list[tuple[float, float]]:
    frame = getattr(getattr(msg, "header", None), "frame_id", "")
    points: list[tuple[float, float]] = []
    for pose_stamped in msg.poses:
        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        if frame in ("nav_center", "base_link", "") and pose is not None:
            rx, ry, yaw = pose
            x, y = (
                rx + x * math.cos(yaw) - y * math.sin(yaw),
                ry + x * math.sin(yaw) + y * math.cos(yaw),
            )
        points.append((x, y))
    return points


def y_at_x(points: list[tuple[float, float]], target_x: float) -> float | None:
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        if (ax <= target_x <= bx) or (bx <= target_x <= ax):
            if abs(bx - ax) < 1e-9:
                return 0.5 * (ay + by)
            t = (target_x - ax) / (bx - ax)
            return ay + t * (by - ay)
    return None


def grid_info(msg):
    if hasattr(msg, "metadata"):
        meta = msg.metadata
        return (
            int(meta.size_x),
            int(meta.size_y),
            float(meta.resolution),
            float(meta.origin.position.x),
            float(meta.origin.position.y),
        )
    info = msg.info
    return (
        int(info.width),
        int(info.height),
        float(info.resolution),
        float(info.origin.position.x),
        float(info.origin.position.y),
    )


def grid_value(msg, x: float, y: float) -> int | None:
    width, height, res, origin_x, origin_y = grid_info(msg)
    if res <= 0.0:
        return None
    mx = int(math.floor((x - origin_x) / res))
    my = int(math.floor((y - origin_y) / res))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    value = int(msg.data[my * width + mx])
    return None if value < 0 else value


def grid_max_near(msg, x: float, y: float, radius_m: float) -> int | None:
    width, height, res, origin_x, origin_y = grid_info(msg)
    if res <= 0.0:
        return None
    mx = int(math.floor((x - origin_x) / res))
    my = int(math.floor((y - origin_y) / res))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    radius_cells = max(0, int(math.ceil(radius_m / res)))
    best: int | None = None
    for ny in range(max(0, my - radius_cells),
                    min(height, my + radius_cells + 1)):
        cy = origin_y + (ny + 0.5) * res
        for nx in range(max(0, mx - radius_cells),
                        min(width, mx + radius_cells + 1)):
            cx = origin_x + (nx + 0.5) * res
            if math.hypot(cx - x, cy - y) > radius_m + 0.5 * res:
                continue
            value = int(msg.data[ny * width + nx])
            if value < 0:
                continue
            best = value if best is None else max(best, value)
    return best


def bag_messages(bag_path: str, topic_filter: set[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = {
        topic: get_message(topic_types[topic])
        for topic in topic_filter
        if topic in topic_types
    }
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg_type = selected.get(topic)
        if msg_type is None:
            continue
        yield topic, deserialize_message(data, msg_type), stamp_ns * 1e-9


def find_named_segment(values: dict[str, object],
                       prefix: str,
                       name_substring: str,
                       fallback_index: int) -> tuple[str, tuple[float, float],
                                                     tuple[float, float]]:
    count = int(values.get(f"{prefix}_count", 0))
    selected = fallback_index
    for idx in range(count):
        name = cfg_str(values, f"{prefix}_{idx}_name", f"{prefix}_{idx}")
        if name_substring in name:
            selected = idx
            break
    base = f"{prefix}_{selected}_"
    return (
        cfg_str(values, f"{base}name", f"{prefix}_{selected}"),
        (cfg_float(values, f"{base}start_x_m"),
         cfg_float(values, f"{base}start_y_m")),
        (cfg_float(values, f"{base}end_x_m"),
         cfg_float(values, f"{base}end_y_m")),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--scenario-config", required=True)
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--gap-samples", type=int, default=25)
    parser.add_argument("--local-hard-threshold", type=int, default=100)
    parser.add_argument("--global-hard-threshold", type=int, default=254)
    parser.add_argument("--line-mask-threshold", type=int, default=1)
    parser.add_argument("--exact-line-threshold", type=int, default=100)
    parser.add_argument("--global-lookback-sec", type=float, default=0.10)
    parser.add_argument("--global-lag-sec", type=float, default=0.75)
    parser.add_argument("--global-hard-search-radius-m", type=float, default=0.08)
    parser.add_argument("--fail-on-gap-plan", action="store_true")
    parser.add_argument("--fail-on-masked-wall", action="store_true")
    args = parser.parse_args()

    values = load_flat_config(Path(args.scenario_config).expanduser())
    scenario_id = cfg_str(values, "scenario_id", Path(args.scenario_config).stem)
    tape_name, tape_start, tape_end = find_named_segment(
        values, "tape", "diagonal", int(values.get("tape_count", 1)) - 1)
    wall_name, wall_start, wall_end = find_named_segment(
        values, "wall", "right", 0)

    if abs(wall_end[1] - wall_start[1]) > 1e-6:
        raise SystemExit("right wall must be horizontal for this analyzer")
    wall_y = wall_start[1]

    def tape_y_at(x: float) -> float:
        tx0, ty0 = tape_start
        tx1, ty1 = tape_end
        return ty0 + ((x - tx0) / (tx1 - tx0)) * (ty1 - ty0)

    x0, x1 = sorted((tape_start[0], tape_end[0]))
    sample_count = max(2, args.gap_samples)
    gap_xs = [
        x0 + (x1 - x0) * idx / float(sample_count - 1)
        for idx in range(sample_count)
    ]

    odom_times: list[float] = []
    odom_poses: list[tuple[float, float, float]] = []
    nav_events: list[tuple[float, int]] = []
    plans = []
    line_costmaps = []
    local_costmaps = []
    global_costmaps = []
    bag_start = None

    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        if topic == "/local_ekf/odom":
            odom_times.append(stamp)
            odom_poses.append(pose_from_odom(msg, args.nav_center_x))
        elif topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                nav_events.append((stamp, int(status.status)))
        elif topic == "/plan":
            plans.append((stamp, msg))
        elif topic == "/lidar_line_costmap":
            line_costmaps.append((stamp, msg))
        elif topic == "/local_costmap/costmap":
            local_costmaps.append((stamp, msg))
        elif topic == "/global_costmap/costmap_raw":
            global_costmaps.append((stamp, msg))

    if bag_start is None:
        raise SystemExit("empty bag")

    exec_times = [stamp for stamp, state in nav_events if state == 2]
    terminal_times = [stamp for stamp, state in nav_events if state in TERMINAL]
    action_start = min(exec_times) if exec_times else bag_start
    action_end = min(
        (stamp for stamp in terminal_times if stamp >= action_start),
        default=max([stamp for stamp, _ in plans], default=action_start),
    )
    nav_final = nav_events[-1][1] if nav_events else None

    gap_plan_hits = []
    y_ranges: dict[float, list[float]] = {x: [] for x in gap_xs}
    for stamp, msg in plans:
        if stamp < action_start or stamp > action_end or not msg.poses:
            continue
        pose = nearest_pose(odom_times, odom_poses, stamp)
        points = path_points(msg, pose)
        for x in gap_xs:
            plan_y = y_at_x(points, x)
            if plan_y is None:
                continue
            y_ranges[x].append(plan_y)
            line_y = tape_y_at(x)
            y_min, y_max = sorted((wall_y, line_y))
            if y_min <= plan_y <= y_max:
                gap_plan_hits.append((stamp, x, plan_y, y_min, y_max))

    soft_line_wall_loss_samples = []
    exact_overlap_samples = 0
    evidence_samples = 0
    global_times = [entry[0] for entry in global_costmaps]
    for stamp, local in local_costmaps:
        if stamp < action_start or stamp > action_end:
            continue
        line_entry = preceding_entry(line_costmaps, stamp)
        global_window = entries_in_window(
            global_costmaps,
            global_times,
            stamp - args.global_lookback_sec,
            stamp + args.global_lag_sec,
        )
        if line_entry is None or not global_window:
            continue
        _, line_msg = line_entry
        for x in gap_xs:
            line_y = tape_y_at(x)
            probe_y = wall_y
            line_value = grid_value(line_msg, x, probe_y)
            local_value = grid_value(local, x, probe_y)
            global_values = [
                grid_max_near(
                    global_msg,
                    x,
                    probe_y,
                    args.global_hard_search_radius_m,
                )
                for _, global_msg in global_window
            ]
            global_hard_values = [
                value for value in global_values
                if value is not None and value >= args.global_hard_threshold
            ]
            global_value = max(
                (value for value in global_values if value is not None),
                default=None,
            )
            line_masked = (
                line_value is not None
                and line_value >= args.line_mask_threshold
            )
            soft_line_masked = (
                line_value is not None
                and args.line_mask_threshold <= line_value
                and line_value < args.exact_line_threshold
            )
            exact_line_masked = (
                line_value is not None
                and line_value >= args.exact_line_threshold
            )
            local_wall = (
                local_value is not None
                and local_value >= args.local_hard_threshold
            )
            global_wall = (
                bool(global_hard_values)
            )
            if local_wall:
                evidence_samples += 1
            if local_wall and exact_line_masked and line_masked:
                exact_overlap_samples += 1
            if local_wall and soft_line_masked and not global_wall:
                soft_line_wall_loss_samples.append(
                    (stamp, x, probe_y, line_y, local_value, line_value,
                     global_value))

    print("Driveway wall-gap repro analysis")
    print(f"bag: {args.bag}")
    print(f"scenario: {scenario_id}")
    print(
        f"wall: {wall_name} y={wall_y:+.4f}; tape: {tape_name} "
        f"start=({tape_start[0]:+.3f},{tape_start[1]:+.4f}) "
        f"end=({tape_end[0]:+.3f},{tape_end[1]:+.4f})"
    )
    print(
        f"action_window={action_start - bag_start:.2f}s.."
        f"{action_end - bag_start:.2f}s nav_final={nav_final}"
    )

    print("\nplan gap scan")
    print(f"plans={len(plans)} gap_plan_hits={len(gap_plan_hits)}")
    for x in gap_xs[::max(1, len(gap_xs) // 6)]:
        vals = y_ranges[x]
        if not vals:
            print(
                f"  x={x:.3f} gap=[{wall_y:+.3f},{tape_y_at(x):+.3f}] "
                "plan_y=none"
            )
            continue
        print(
            f"  x={x:.3f} gap=[{wall_y:+.3f},{tape_y_at(x):+.3f}] "
            f"plan_y_range=[{min(vals):+.3f},{max(vals):+.3f}]"
        )
    if gap_plan_hits:
        first = gap_plan_hits[0]
        print(
            f"first_gap_plan_hit: t={first[0] - bag_start:.2f}s "
            f"x={first[1]:+.3f} y={first[2]:+.3f} "
            f"gap=[{first[3]:+.3f},{first[4]:+.3f}]"
        )

    print("\nmirror masking scan")
    print(
        f"local_wall_samples={evidence_samples} "
        f"soft_line_wall_loss_samples={len(soft_line_wall_loss_samples)} "
        f"exact_line_wall_overlap_samples={exact_overlap_samples}"
    )
    if soft_line_wall_loss_samples:
        first = soft_line_wall_loss_samples[0]
        global_text = "none" if first[6] is None else str(first[6])
        print(
            f"first_soft_line_wall_loss_sample: t={first[0] - bag_start:.2f}s "
            f"x={first[1]:+.3f} wall_y={first[2]:+.3f} "
            f"line_y={first[3]:+.3f} local={first[4]} "
            f"line_mask={first[5]} global={global_text}"
        )

    failures = []
    if args.fail_on_gap_plan and gap_plan_hits:
        failures.append(f"/plan entered wall-line gap {len(gap_plan_hits)} times")
    if args.fail_on_masked_wall and soft_line_wall_loss_samples:
        failures.append(
            f"local wall cells absent from raw global costmap under soft line cost "
            f"{len(soft_line_wall_loss_samples)} times")
    if failures:
        print("\nFAIL: driveway wall-gap repro analysis")
        for failure in failures:
            print(f"  {failure}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
