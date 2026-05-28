#!/usr/bin/env python3
"""Check global plans against the raw global costmap and robot footprint.

This analyzer is for the lidar-line avoidance test. It time-aligns each
published global path with the nearest preceding /global_costmap/costmap_raw
sample and reports whether the rectangular nav_center footprint would overlap
raw lethal cells. It also reports inscribed-cell clearance as a diagnostic so
we can see how close the plan is to the global inflation field without
double-counting inflation as a footprint collision.
"""

from __future__ import annotations

import argparse
import bisect
import math
from typing import Iterable


TOPICS = {
    "/local_ekf/odom",
    "/goal_pose",
    "/navigate_to_pose/_action/status",
    "/global_costmap/costmap_raw",
    "/plan",
    "/unsmoothed_plan",
}


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_from_odom(msg, nav_center_x):
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    return p.x + nav_center_x * math.cos(yaw), p.y + nav_center_x * math.sin(yaw), yaw


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def nearest_pose(times, poses, stamp):
    if not times:
        return None
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return poses[0]
    if idx >= len(times):
        return poses[-1]
    return poses[idx - 1] if abs(times[idx - 1] - stamp) <= abs(times[idx] - stamp) else poses[idx]


def preceding_entry(entries, stamp):
    if not entries:
        return None
    times = [entry[0] for entry in entries]
    idx = bisect.bisect_right(times, stamp) - 1
    if idx < 0:
        return None
    return entries[idx]


def to_start_frame(start, x, y):
    sx, sy, syaw = start
    dx = x - sx
    dy = y - sy
    c = math.cos(syaw)
    s = math.sin(syaw)
    return c * dx + s * dy, -s * dx + c * dy


def point_to_start(frame, start, pose, x, y):
    if frame in ("map", "odom"):
        return to_start_frame(start, x, y)
    if frame in ("nav_center", "base_link", ""):
        if pose is None:
            return x, y
        rx, ry, ryaw = pose
        wx = rx + x * math.cos(ryaw) - y * math.sin(ryaw)
        wy = ry + x * math.sin(ryaw) + y * math.cos(ryaw)
        return to_start_frame(start, wx, wy)
    return to_start_frame(start, x, y)


def pose_to_start(frame, start, pose, x, y, yaw):
    if frame in ("map", "odom"):
        sx, sy = to_start_frame(start, x, y)
        return sx, sy, angle_diff(yaw, start[2])
    if frame in ("nav_center", "base_link", ""):
        if pose is None:
            return x, y, yaw
        rx, ry, ryaw = pose
        wx = rx + x * math.cos(ryaw) - y * math.sin(ryaw)
        wy = ry + x * math.sin(ryaw) + y * math.cos(ryaw)
        sx, sy = to_start_frame(start, wx, wy)
        return sx, sy, angle_diff(ryaw + yaw, start[2])
    sx, sy = to_start_frame(start, x, y)
    return sx, sy, angle_diff(yaw, start[2])


def y_at_x(path, target_x):
    for a, b in zip(path, path[1:]):
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        if (ax <= target_x <= bx) or (bx <= target_x <= ax):
            if abs(bx - ax) < 1e-6:
                return 0.5 * (ay + by)
            ratio = (target_x - ax) / (bx - ax)
            return ay + ratio * (by - ay)
    return None


def path_len(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def signed_box_clearance(x, y, half_x, half_y):
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


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


def frame_id(msg):
    return getattr(getattr(msg, "header", None), "frame_id", "")


def threshold_for_msg(msg, threshold):
    if hasattr(msg, "metadata"):
        return threshold
    if threshold > 100:
        return max(1, int(round(threshold * 100.0 / 254.0)))
    return threshold


def costmap_cells_to_start(msg, threshold, unknown_value, start, pose):
    width, height, res, origin_x, origin_y = grid_info(msg)
    effective_threshold = threshold_for_msg(msg, threshold)
    frame = frame_id(msg)
    cells = []
    for iy in range(height):
        row = iy * width
        y = origin_y + (iy + 0.5) * res
        for ix in range(width):
            value = int(msg.data[row + ix])
            if value == unknown_value or value < 0:
                continue
            if value < effective_threshold:
                continue
            x = origin_x + (ix + 0.5) * res
            sx, sy = point_to_start(frame, start, pose, x, y)
            cells.append((sx, sy, value))
    return cells


def path_to_start(msg, start, pose):
    frame = frame_id(msg)
    path = []
    for pose_stamped in msg.poses:
        p = pose_stamped.pose.position
        yaw = yaw_from_quat(pose_stamped.pose.orientation)
        path.append(pose_to_start(frame, start, pose, p.x, p.y, yaw))
    return path


def min_center_distance(path, cells):
    if not path or not cells:
        return None
    best = None
    for px, py, _ in path:
        for cx, cy, _ in cells:
            dist = math.hypot(cx - px, cy - py)
            if best is None or dist < best:
                best = dist
    return best


def min_footprint_clearance(path, cells, half_length, half_width):
    if not path or not cells:
        return None
    best = None
    for pose_index, (px, py, yaw) in enumerate(path):
        c = math.cos(yaw)
        s = math.sin(yaw)
        for cx, cy, value in cells:
            dx = cx - px
            dy = cy - py
            rel_x = c * dx + s * dy
            rel_y = -s * dx + c * dy
            clearance = signed_box_clearance(rel_x, rel_y, half_length, half_width)
            if best is None or clearance < best[0]:
                best = (clearance, pose_index, rel_x, rel_y, value)
    return best


def crosses_measured_tape(y_cross, y_min, y_max):
    return y_cross is not None and y_min <= y_cross <= y_max


def fmt_float(value, digits=3):
    if value is None:
        return "none"
    return f"{value:+.{digits}f}"


def fmt_bool(value):
    return "yes" if value else "no"


def bag_messages(bag_path, topics: Iterable[str]):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected = {topic: get_message(topic_types[topic]) for topic in topics if topic in topic_types}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg_type = selected.get(topic)
        if msg_type is None:
            continue
        yield topic, deserialize_message(data, msg_type), stamp_ns * 1e-9


def summarize_path(topic, stamp, path, grid_entry, args, bag_start):
    rel_t = stamp - bag_start
    if grid_entry is None:
        print(f"  {topic} t={rel_t:.2f}s no preceding /global_costmap/costmap_raw")
        return None

    grid_stamp, grid_msg = grid_entry
    y_cross = y_at_x(path, args.perp_x)
    center_on_tape = crosses_measured_tape(y_cross, args.perp_y_min, args.perp_y_max)
    gap_center_ok = y_cross is not None and y_cross <= args.tape_right_y - args.half_width
    grid_pose = None
    lethal_cells = costmap_cells_to_start(
        grid_msg, args.lethal_threshold, args.unknown_value, args.start_pose, grid_pose)
    inscribed_cells = costmap_cells_to_start(
        grid_msg, args.inscribed_threshold, args.unknown_value, args.start_pose, grid_pose)

    lethal_center = min_center_distance(path, lethal_cells)
    inscribed_center = min_center_distance(path, inscribed_cells)
    lethal_clearance = min_footprint_clearance(
        path, lethal_cells, args.half_length, args.half_width)
    inscribed_clearance = min_footprint_clearance(
        path, inscribed_cells, args.half_length, args.half_width)

    lethal_clear = None if lethal_clearance is None else lethal_clearance[0]
    inscribed_clear = None if inscribed_clearance is None else inscribed_clearance[0]
    print(
        f"  {topic} t={rel_t:.2f}s age={stamp - grid_stamp:+.2f}s poses={len(path)} "
        f"len={path_len(path):.2f} y_at_perp={fmt_float(y_cross)} "
        f"center_on_tape={fmt_bool(center_on_tape)} gap_center_ok={fmt_bool(gap_center_ok)} "
        f"lethal_cells={len(lethal_cells)} min_center_lethal={fmt_float(lethal_center)} "
        f"footprint_lethal_clear={fmt_float(lethal_clear)} "
        f"lethal_overlap={fmt_bool(lethal_clear is not None and lethal_clear <= 0.0)} "
        f"inscribed_cells={len(inscribed_cells)} min_center_inscribed={fmt_float(inscribed_center)} "
        f"footprint_inscribed_clear={fmt_float(inscribed_clear)} "
        f"inscribed_overlap={fmt_bool(inscribed_clear is not None and inscribed_clear <= 0.0)}"
    )
    return {
        "topic": topic,
        "stamp": stamp,
        "y_cross": y_cross,
        "lethal_clear": lethal_clear,
        "inscribed_clear": inscribed_clear,
        "gap_center_ok": gap_center_ok,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--half-length", type=float, default=0.545)
    parser.add_argument("--half-width", type=float, default=0.41)
    parser.add_argument("--lethal-threshold", type=int, default=254)
    parser.add_argument("--inscribed-threshold", type=int, default=253)
    parser.add_argument("--unknown-value", type=int, default=255)
    parser.add_argument("--perp-x", type=float, default=1.34)
    parser.add_argument("--perp-y-min", type=float, default=-0.13)
    parser.add_argument("--perp-y-max", type=float, default=0.50)
    parser.add_argument("--tape-right-y", type=float, default=-0.13)
    parser.add_argument(
        "--require-plan",
        action="store_true",
        help="Exit nonzero if no /plan or /unsmoothed_plan is available after action start.",
    )
    parser.add_argument(
        "--fail-on-overlap",
        action="store_true",
        help="Exit nonzero if any analyzed plan footprint overlaps lethal raw global cells.",
    )
    parser.add_argument(
        "--fail-on-inscribed-overlap",
        action="store_true",
        help="Also exit nonzero if any analyzed plan footprint overlaps inscribed raw global cells.",
    )
    parser.add_argument(
        "--clearance-margin",
        type=float,
        default=0.0,
        help="Minimum acceptable signed footprint clearance for enabled overlap failures.",
    )
    parser.add_argument(
        "--fail-on-gap-center",
        action="store_true",
        help="Exit nonzero if any /plan centerline at the measured tape is not through the configured gap.",
    )
    args = parser.parse_args()

    odom_times = []
    odom_poses = []
    goal_time = None
    nav_exec_time = None
    costmaps = []
    paths = {"/plan": [], "/unsmoothed_plan": []}
    bag_start = None

    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        if topic == "/local_ekf/odom":
            odom_times.append(stamp)
            odom_poses.append(pose_from_odom(msg, args.nav_center_x))
        elif topic == "/goal_pose":
            goal_time = stamp
        elif topic == "/navigate_to_pose/_action/status" and nav_exec_time is None:
            for status in msg.status_list:
                if int(status.status) == 2:
                    nav_exec_time = stamp
                    break
        elif topic == "/global_costmap/costmap_raw":
            costmaps.append((stamp, msg))
        elif topic in paths:
            paths[topic].append((stamp, msg))

    if bag_start is None:
        raise SystemExit("empty bag")
    if not odom_poses:
        raise SystemExit("no /local_ekf/odom samples")

    action_start = goal_time or nav_exec_time or odom_times[0]
    start_pose = nearest_pose(odom_times, odom_poses, action_start)
    if start_pose is None:
        raise SystemExit("no odom pose for action start")
    args.start_pose = start_pose

    print("Global plan raw-costmap footprint analysis")
    print(f"bag: {args.bag}")
    print(f"action_start={action_start - bag_start:.2f}s costmap_samples={len(costmaps)}")
    print(
        f"footprint half_length={args.half_length:.3f} half_width={args.half_width:.3f} "
        f"lethal_threshold={args.lethal_threshold} inscribed_threshold={args.inscribed_threshold}"
    )
    print(
        f"measured perpendicular tape x={args.perp_x:.3f} "
        f"y=[{args.perp_y_min:+.3f},{args.perp_y_max:+.3f}], "
        f"gap-center pass target y<={args.tape_right_y - args.half_width:+.3f}"
    )

    summaries = {"/plan": [], "/unsmoothed_plan": []}
    for topic in ("/unsmoothed_plan", "/plan"):
        print(f"\n{topic}")
        considered = 0
        for stamp, msg in paths[topic]:
            if stamp < action_start:
                continue
            pose = nearest_pose(odom_times, odom_poses, stamp)
            path = path_to_start(msg, start_pose, pose)
            if not path:
                continue
            considered += 1
            summary = summarize_path(
                topic,
                stamp,
                path,
                preceding_entry(costmaps, stamp),
                args,
                bag_start,
            )
            if summary is not None:
                summaries[topic].append(summary)
        if considered == 0:
            print("  none after action start")

    if summaries["/plan"] and summaries["/unsmoothed_plan"]:
        print("\nsmoothed-vs-unsmoothed nearest-time comparison")
        unsmoothed_times = [item["stamp"] for item in summaries["/unsmoothed_plan"]]
        for plan in summaries["/plan"]:
            idx = bisect.bisect_left(unsmoothed_times, plan["stamp"])
            candidates = []
            if idx > 0:
                candidates.append(summaries["/unsmoothed_plan"][idx - 1])
            if idx < len(summaries["/unsmoothed_plan"]):
                candidates.append(summaries["/unsmoothed_plan"][idx])
            if not candidates:
                continue
            raw = min(candidates, key=lambda item: abs(item["stamp"] - plan["stamp"]))
            if abs(raw["stamp"] - plan["stamp"]) > 0.5:
                continue
            y_delta = None
            if plan["y_cross"] is not None and raw["y_cross"] is not None:
                y_delta = plan["y_cross"] - raw["y_cross"]
            lethal_delta = None
            if plan["lethal_clear"] is not None and raw["lethal_clear"] is not None:
                lethal_delta = plan["lethal_clear"] - raw["lethal_clear"]
            print(
                f"  plan_t={plan['stamp'] - bag_start:.2f}s raw_t={raw['stamp'] - bag_start:.2f}s "
                f"dy_at_perp={fmt_float(y_delta)} "
                f"d_footprint_lethal_clear={fmt_float(lethal_delta)}"
            )

    failures = []
    if args.require_plan:
        for topic in ("/unsmoothed_plan", "/plan"):
            if not summaries[topic]:
                failures.append(f"{topic}: no plans after action start")

    if args.fail_on_overlap or args.fail_on_inscribed_overlap:
        for topic in ("/unsmoothed_plan", "/plan"):
            for summary in summaries[topic]:
                rel_t = summary["stamp"] - bag_start
                lethal_clear = summary["lethal_clear"]
                inscribed_clear = summary["inscribed_clear"]
                if (
                    args.fail_on_overlap and
                    lethal_clear is not None and
                    lethal_clear <= args.clearance_margin
                ):
                    failures.append(
                        f"{topic} t={rel_t:.2f}s lethal clearance {lethal_clear:+.3f} "
                        f"<= margin {args.clearance_margin:+.3f}"
                    )
                if (
                    args.fail_on_inscribed_overlap and
                    inscribed_clear is not None and
                    inscribed_clear <= args.clearance_margin
                ):
                    failures.append(
                        f"{topic} t={rel_t:.2f}s inscribed clearance {inscribed_clear:+.3f} "
                        f"<= margin {args.clearance_margin:+.3f}"
                    )

    if args.fail_on_gap_center:
        for summary in summaries["/plan"]:
            if not summary["gap_center_ok"]:
                rel_t = summary["stamp"] - bag_start
                failures.append(
                    f"/plan t={rel_t:.2f}s y_at_perp={fmt_float(summary['y_cross'])} "
                    f"is not through gap target"
                )

    if failures:
        print("\nFAIL: unsafe global plan geometry")
        for failure in failures[:20]:
            print(f"  {failure}")
        if len(failures) > 20:
            print(f"  ... {len(failures) - 20} more")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
