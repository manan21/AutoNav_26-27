#!/usr/bin/env python3
"""Check hard costmap cells against the nav_center footprint in a ROS 2 bag."""

from __future__ import annotations

import argparse
import bisect
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/local_ekf/odom",
    "/local_costmap/costmap",
    "/lidar_line_costmap",
    "/line_costmap",
}


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def nav_center_pose_from_odom(msg, nav_center_x):
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    return (
        p.x + nav_center_x * math.cos(yaw),
        p.y + nav_center_x * math.sin(yaw),
        yaw,
    )


def signed_box_clearance(x, y, half_x, half_y):
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


def bag_messages(bag_path, topic_filter):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = {topic: get_message(topic_types[topic]) for topic in topic_filter if topic in topic_types}
    if not selected:
        raise SystemExit(f"none of {sorted(topic_filter)} found in bag")
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg_type = selected.get(topic)
        if msg_type is None:
            continue
        yield topic, deserialize_message(data, msg_type), stamp / 1e9


def nearest_pose(poses, times, stamp):
    if not times:
        return None
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return poses[0]
    if idx >= len(times):
        return poses[-1]
    return poses[idx - 1] if abs(times[idx - 1] - stamp) <= abs(times[idx] - stamp) else poses[idx]


def summarize_grid(msg, pose, hard_threshold, half_x, half_y):
    rx, ry, ryaw = pose
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    res = msg.info.resolution
    width = msg.info.width
    height = msg.info.height

    hard = 0
    inside = 0
    nearest = None
    nearest_rel = None
    for iy in range(height):
        row = iy * width
        y = origin_y + (iy + 0.5) * res
        for ix in range(width):
            value = msg.data[row + ix]
            if value < hard_threshold:
                continue
            hard += 1
            x = origin_x + (ix + 0.5) * res
            dx = x - rx
            dy = y - ry
            rel_x = c * dx + s * dy
            rel_y = -s * dx + c * dy
            clearance = signed_box_clearance(rel_x, rel_y, half_x, half_y)
            if clearance <= 0.0:
                inside += 1
            if nearest is None or clearance < nearest:
                nearest = clearance
                nearest_rel = (rel_x, rel_y, value)
    return hard, inside, nearest, nearest_rel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--half-length", type=float, default=0.575)
    parser.add_argument("--half-width", type=float, default=0.44)
    parser.add_argument("--hard-threshold", type=int, default=99)
    parser.add_argument("--print-every", type=float, default=1.0)
    args = parser.parse_args()

    odom_times = []
    odom_poses = []
    grids = {topic: [] for topic in TOPICS if topic != "/local_ekf/odom"}
    bag_start = None

    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        if bag_start is None:
            bag_start = stamp
        if topic == "/local_ekf/odom":
            odom_times.append(stamp)
            odom_poses.append(nav_center_pose_from_odom(msg, args.nav_center_x))
        elif topic in grids:
            grids[topic].append((stamp, msg))

    if bag_start is None:
        raise SystemExit("empty bag")

    print("Costmap footprint analysis")
    print(f"bag: {args.bag}")
    print(
        f"footprint half-length={args.half_length:.3f} half-width={args.half_width:.3f} "
        f"hard_threshold={args.hard_threshold}"
    )

    for topic, entries in sorted(grids.items()):
        if not entries:
            continue
        print(f"\n{topic}")
        last_print = None
        worst = None
        for stamp, grid in entries:
            pose = nearest_pose(odom_poses, odom_times, stamp)
            if pose is None:
                continue
            hard, inside, nearest, nearest_rel = summarize_grid(
                grid,
                pose,
                args.hard_threshold,
                args.half_length,
                args.half_width,
            )
            rel_t = stamp - bag_start
            if worst is None or inside > worst[1] or (
                inside == worst[1] and nearest is not None and nearest < worst[2]
            ):
                worst = (rel_t, inside, nearest if nearest is not None else float("inf"), hard, nearest_rel)
            should_print = inside > 0
            if last_print is None or rel_t - last_print >= args.print_every:
                should_print = should_print or (nearest is not None and nearest < 0.15)
            if should_print:
                last_print = rel_t
                rel_txt = "none"
                if nearest_rel is not None:
                    rel_txt = (
                        f"x={nearest_rel[0]:+.3f} y={nearest_rel[1]:+.3f} "
                        f"cost={nearest_rel[2]}"
                    )
                nearest_txt = "none" if nearest is None else f"{nearest:+.3f}"
                print(
                    f"  t={rel_t:6.2f}s hard={hard:5d} inside={inside:4d} "
                    f"nearest_clearance={nearest_txt} nearest[{rel_txt}]"
                )
        if worst is not None:
            rel_txt = "none"
            if worst[4] is not None:
                rel_txt = f"x={worst[4][0]:+.3f} y={worst[4][1]:+.3f} cost={worst[4][2]}"
            print(
                f"  worst: t={worst[0]:.2f}s hard={worst[3]} inside={worst[1]} "
                f"nearest_clearance={worst[2]:+.3f} nearest[{rel_txt}]"
            )


if __name__ == "__main__":
    main()
