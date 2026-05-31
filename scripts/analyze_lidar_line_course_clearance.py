#!/usr/bin/env python3
"""Compare the robot trajectory against measured lidar-line course geometry.

This answers the physical pass/fail question that Nav2 success cannot answer:
did the nav_center trajectory keep the rectangular robot footprint clear of the
measured perpendicular tape obstacle?
"""

from __future__ import annotations

import argparse
import bisect
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {"/local_ekf/odom", "/navigate_to_pose/_action/status"}


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_from_odom(msg, nav_center_x):
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    return p.x + nav_center_x * math.cos(yaw), p.y + nav_center_x * math.sin(yaw), yaw


def rel_pose(start, pose):
    sx, sy, syaw = start
    x, y, yaw = pose
    dx = x - sx
    dy = y - sy
    c = math.cos(syaw)
    s = math.sin(syaw)
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw))


def signed_box_clearance(x, y, half_x, half_y):
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


def nearest_index(times, stamp):
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    return idx - 1 if abs(times[idx - 1] - stamp) <= abs(times[idx] - stamp) else idx


def y_at_x(points, target_x):
    for a, b in zip(points, points[1:]):
        ax, ay = a
        bx, by = b
        if (ax <= target_x <= bx) or (bx <= target_x <= ax):
            if abs(bx - ax) < 1e-6:
                return 0.5 * (ay + by)
            t = (target_x - ax) / (bx - ax)
            return ay + t * (by - ay)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--half-length", type=float, default=0.545)
    parser.add_argument("--half-width", type=float, default=0.41)
    parser.add_argument("--padding", type=float, default=0.03)
    parser.add_argument("--perp-x", type=float, default=1.34)
    parser.add_argument("--perp-y-min", type=float, default=-0.13)
    parser.add_argument("--perp-y-max", type=float, default=1.524)
    parser.add_argument("--sample-step", type=float, default=0.005)
    parser.add_argument(
        "--fail-on-overlap",
        action="store_true",
        help="Exit nonzero if physical or padded footprint clearance is not positive.",
    )
    parser.add_argument(
        "--min-physical-clearance",
        type=float,
        default=0.0,
        help="Minimum acceptable physical footprint clearance when --fail-on-overlap is set.",
    )
    parser.add_argument(
        "--min-padded-clearance",
        type=float,
        default=0.0,
        help="Minimum acceptable padded footprint clearance when --fail-on-overlap is set.",
    )
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(topic_types[topic]) for topic in TOPICS if topic in topic_types}

    odom = []
    nav_events = []
    bag_start = None
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in msg_types:
            continue
        stamp = stamp_ns * 1e-9
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        msg = deserialize_message(data, msg_types[topic])
        if topic == "/local_ekf/odom":
            odom.append((stamp, pose_from_odom(msg, args.nav_center_x)))
        elif topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                nav_events.append((stamp, int(status.status)))

    if not odom:
        raise SystemExit("no /local_ekf/odom samples")
    if not nav_events:
        raise SystemExit("no /navigate_to_pose/_action/status samples")
    if bag_start is None:
        raise SystemExit("empty bag")

    exec_times = [stamp for stamp, status in nav_events if status == 2]
    terminal_times = [stamp for stamp, status in nav_events if status in (4, 5, 6)]
    if not exec_times:
        raise SystemExit("no executing NavigateToPose status")
    start_t = exec_times[0]
    end_t = terminal_times[-1] if terminal_times else odom[-1][0]
    times = [stamp for stamp, _ in odom]
    start_pose = odom[nearest_index(times, start_t)][1]

    rel_samples = []
    for stamp, pose in odom:
        if start_t <= stamp <= end_t:
            rel_samples.append((stamp, rel_pose(start_pose, pose)))
    if not rel_samples:
        raise SystemExit("no odom samples in NavigateToPose execution window")

    traj_xy = [(x, y) for _, (x, y, _) in rel_samples]
    ys = [y for _, y in traj_xy]
    xs = [x for x, _ in traj_xy]
    y_cross = y_at_x(traj_xy, args.perp_x)

    tape_points = []
    n = max(1, int(math.ceil((args.perp_y_max - args.perp_y_min) / args.sample_step)))
    for i in range(n + 1):
        y = args.perp_y_min + (args.perp_y_max - args.perp_y_min) * i / n
        tape_points.append((args.perp_x, y))

    best_physical = None
    best_padded = None
    overlaps_physical = 0
    overlaps_padded = 0
    for stamp, (rx, ry, ryaw) in rel_samples:
        c = math.cos(ryaw)
        s = math.sin(ryaw)
        for px, py in tape_points:
            dx = px - rx
            dy = py - ry
            robot_x = c * dx + s * dy
            robot_y = -s * dx + c * dy
            physical = signed_box_clearance(robot_x, robot_y, args.half_length, args.half_width)
            padded = signed_box_clearance(
                robot_x,
                robot_y,
                args.half_length + args.padding,
                args.half_width + args.padding,
            )
            physical_candidate = (physical, stamp, rx, ry, math.degrees(ryaw), px, py, robot_x, robot_y)
            padded_candidate = (padded, stamp, rx, ry, math.degrees(ryaw), px, py, robot_x, robot_y)
            if best_physical is None or physical_candidate[0] < best_physical[0]:
                best_physical = physical_candidate
            if best_padded is None or padded_candidate[0] < best_padded[0]:
                best_padded = padded_candidate
            if physical < 0.0:
                overlaps_physical += 1
            if padded < 0.0:
                overlaps_padded += 1

    print("Lidar-line measured-course clearance analysis")
    print(f"bag: {args.bag}")
    print(f"window: {start_t - bag_start:.2f}s to {end_t - bag_start:.2f}s")
    print(f"trajectory fwd range=[{min(xs):+.3f},{max(xs):+.3f}] left range=[{min(ys):+.3f},{max(ys):+.3f}]")
    if y_cross is None:
        print("trajectory y at perpendicular tape: none")
    else:
        print(f"trajectory y at perpendicular tape x={args.perp_x:.3f}: {y_cross:+.3f}")

    for label, best, overlaps in (
        ("physical", best_physical, overlaps_physical),
        ("padded", best_padded, overlaps_padded),
    ):
        clearance, stamp, rx, ry, yaw, px, py, robot_x, robot_y = best
        print(
            f"{label} footprint min clearance to perpendicular tape: {clearance:+.3f} m "
            f"at t={stamp - start_t:.2f}s pose=({rx:+.3f},{ry:+.3f},{yaw:+.1f}deg) "
            f"tape=({px:+.3f},{py:+.3f}) robot_frame=({robot_x:+.3f},{robot_y:+.3f}) "
            f"sample_overlaps={overlaps}"
        )

    failures = []
    if args.fail_on_overlap:
        physical_clearance = best_physical[0]
        padded_clearance = best_padded[0]
        if physical_clearance <= args.min_physical_clearance:
            failures.append(
                f"physical clearance {physical_clearance:+.3f} "
                f"<= {args.min_physical_clearance:+.3f}"
            )
        if padded_clearance <= args.min_padded_clearance:
            failures.append(
                f"padded clearance {padded_clearance:+.3f} "
                f"<= {args.min_padded_clearance:+.3f}"
            )
    if failures:
        print("\nFAIL: measured-course tape clearance")
        for failure in failures:
            print(f"  {failure}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
