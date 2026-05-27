#!/usr/bin/env python3
"""Check whether lidar-line test global plans route through the measured gap.

Run inside the robot ROS 2 environment after sourcing the workspace:

    python3 scripts/analyze_lidar_line_plan_gap.py /path/to/bag

Coordinates are reported in the nav-center action-start frame used by the
lidar-line test analysis. The defaults match docs/LIDAR_LINE_AVOIDANCE_COURSE.md.
"""

from __future__ import annotations

import argparse
import bisect
import math
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/local_ekf/odom",
    "/goal_pose",
    "/plan",
    "/lidar_line_points",
    "/lidar_line_costmap",
    "/global_costmap/costmap",
    "/autonomous_mode",
    "/navigate_to_pose/_action/status",
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


def nearest_pose(times, poses, stamp):
    if not times:
        return None
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return poses[0]
    if idx >= len(times):
        return poses[-1]
    return poses[idx - 1] if abs(times[idx - 1] - stamp) <= abs(times[idx] - stamp) else poses[idx]


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


def path_len(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def min_dist_to_cells(points, cells):
    if not points or not cells:
        return None
    best = None
    for px, py in points:
        for cx, cy in cells:
            dist = math.hypot(px - cx, py - cy)
            if best is None or dist < best:
                best = dist
    return best


def y_at_x(points, target_x):
    if len(points) < 2:
        return None
    candidates = []
    for a, b in zip(points, points[1:]):
        ax, ay = a
        bx, by = b
        if (ax <= target_x <= bx) or (bx <= target_x <= ax):
            if abs(bx - ax) < 1e-6:
                candidates.append((ay + by) * 0.5)
            else:
                t = (target_x - ax) / (bx - ax)
                candidates.append(ay + t * (by - ay))
    return candidates[0] if candidates else None


def summarize_points(points):
    if not points:
        return "none"
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return f"n={len(points)} x=[{min(xs):+.3f},{max(xs):+.3f}] y=[{min(ys):+.3f},{max(ys):+.3f}]"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--hard-threshold", type=int, default=99)
    parser.add_argument("--perp-x", type=float, default=1.34)
    parser.add_argument("--tape-right-y", type=float, default=-0.13)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(topic_types[topic]) for topic in TOPICS if topic in topic_types}

    raw = []
    bag_start = None
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in msg_types:
            continue
        stamp = stamp_ns * 1e-9
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        raw.append((topic, deserialize_message(data, msg_types[topic]), stamp))

    if bag_start is None:
        raise SystemExit("empty bag")

    odom_times = []
    odom_poses = []
    goal_time = None
    auto_modes = []
    for topic, msg, stamp in raw:
        if topic == "/local_ekf/odom":
            odom_times.append(stamp)
            odom_poses.append(pose_from_odom(msg, args.nav_center_x))
        elif topic == "/goal_pose":
            goal_time = stamp
        elif topic == "/autonomous_mode":
            auto_modes.append((stamp - bag_start, bool(msg.data)))
        elif topic == "/navigate_to_pose/_action/status" and goal_time is None:
            for status in msg.status_list:
                if int(status.status) == 2:
                    goal_time = stamp
                    break

    if not odom_poses:
        raise SystemExit("no /local_ekf/odom samples")

    start = nearest_pose(odom_times, odom_poses, goal_time or odom_times[0])
    print("Lidar-line global plan gap analysis")
    print(f"bag: {args.bag}")
    print("goal_time: none" if goal_time is None else f"goal_time: {goal_time - bag_start:.2f}s")
    print(f"autonomous_mode samples: {auto_modes or 'none'}")

    line_points_after_goal = []
    first_line_summary = None
    last_line_summary = None
    for topic, msg, stamp in raw:
        if topic != "/lidar_line_points" or (goal_time is not None and stamp < goal_time):
            continue
        pose = nearest_pose(odom_times, odom_poses, stamp)
        pts = [point_to_start(msg.header.frame_id, start, pose, p.x, p.y) for p in msg.points]
        if pts:
            summary = (stamp - bag_start, pts)
            first_line_summary = first_line_summary or summary
            last_line_summary = summary
            line_points_after_goal.extend(pts)

    print("\nlidar_line_points after goal")
    if first_line_summary:
        print(f"first t={first_line_summary[0]:.2f}s {summarize_points(first_line_summary[1])}")
        print(f"last  t={last_line_summary[0]:.2f}s {summarize_points(last_line_summary[1])}")
        print(f"all   {summarize_points(line_points_after_goal)}")
    else:
        print("none")

    hard_by_topic = {}
    for grid_topic in ("/lidar_line_costmap", "/global_costmap/costmap"):
        latest_cells = []
        first_cells = None
        samples = 0
        nonempty = 0
        for topic, msg, stamp in raw:
            if topic != grid_topic or (goal_time is not None and stamp < goal_time):
                continue
            pose = nearest_pose(odom_times, odom_poses, stamp)
            cells = []
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            res = msg.info.resolution
            width = msg.info.width
            height = msg.info.height
            for iy in range(height):
                row = iy * width
                y = origin_y + (iy + 0.5) * res
                for ix in range(width):
                    if msg.data[row + ix] >= args.hard_threshold:
                        x = origin_x + (ix + 0.5) * res
                        cells.append(point_to_start(msg.header.frame_id, start, pose, x, y))
            samples += 1
            if cells:
                nonempty += 1
                first_cells = first_cells or (stamp - bag_start, cells)
                latest_cells = cells
        hard_by_topic[grid_topic] = latest_cells
        print(f"\n{grid_topic} hard cells after goal")
        print(f"samples={samples} nonempty={nonempty}")
        if first_cells:
            print(f"first t={first_cells[0]:.2f}s {summarize_points(first_cells[1])}")
            print(f"last  {summarize_points(latest_cells)}")
        else:
            print("none")

    plans = []
    for topic, msg, stamp in raw:
        if topic != "/plan" or (goal_time is not None and stamp < goal_time):
            continue
        pose = nearest_pose(odom_times, odom_poses, stamp)
        pts = [
            point_to_start(msg.header.frame_id, start, pose, p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]
        if pts:
            plans.append((stamp - bag_start, pts))

    print("\nglobal plans")
    print(f"count={len(plans)}")
    for label, item in (("first", plans[0] if plans else None), ("last", plans[-1] if plans else None)):
        if item is None:
            continue
        rel_t, pts = item
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        y_perp = y_at_x(pts, args.perp_x)
        min_line = min_dist_to_cells(pts, hard_by_topic.get("/lidar_line_costmap", []))
        min_global = min_dist_to_cells(pts, hard_by_topic.get("/global_costmap/costmap", []))
        print(
            f"{label} t={rel_t:.2f}s poses={len(pts)} len={path_len(pts):.3f} "
            f"x=[{min(xs):+.3f},{max(xs):+.3f}] y=[{min(ys):+.3f},{max(ys):+.3f}] "
            f"y_at_perp_x={y_perp if y_perp is not None else 'none'} "
            f"min_to_lidar_line={min_line if min_line is not None else 'none'} "
            f"min_to_global_hard={min_global if min_global is not None else 'none'}"
        )

    if plans:
        y_at_perps = [y_at_x(pts, args.perp_x) for _, pts in plans]
        y_at_perps = [y for y in y_at_perps if y is not None]
        max_abs_y = [max(abs(y) for _, y in pts) for _, pts in plans]
        min_to_line = [
            min_dist_to_cells(pts, hard_by_topic.get("/lidar_line_costmap", []))
            for _, pts in plans
        ]
        min_to_line = [dist for dist in min_to_line if dist is not None]
        if y_at_perps:
            print(
                f"plan y at measured perpendicular x={args.perp_x:.2f}m: "
                f"median={statistics.median(y_at_perps):+.3f} "
                f"range=[{min(y_at_perps):+.3f},{max(y_at_perps):+.3f}]"
            )
            print(
                f"measured tape right end y={args.tape_right_y:+.3f}m; "
                "a route through the right-side gap should be below that end with footprint clearance"
            )
        print(
            f"plan max |lateral| median/range="
            f"{statistics.median(max_abs_y):.3f}/[{min(max_abs_y):.3f},{max(max_abs_y):.3f}]"
        )
        if min_to_line:
            print(
                f"plan min distance to lidar-line hard cells median/range="
                f"{statistics.median(min_to_line):.3f}/[{min(min_to_line):.3f},{max(min_to_line):.3f}]"
            )


if __name__ == "__main__":
    main()
