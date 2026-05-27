#!/usr/bin/env python3
"""Print a compact lidar-line test timeline from a ROS 2 bag."""

from __future__ import annotations

import argparse
import bisect
import math
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/local_ekf/odom",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/lidar_line_points",
    "/plan",
    "/evaluation",
    "/navigate_to_pose/_action/status",
    "/follow_path/_action/status",
}

STATUS = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def nav_pose(msg, nav_center_x=0.225):
    p = msg.pose.pose.position
    yaw = yaw_from_q(msg.pose.pose.orientation)
    return p.x + nav_center_x * math.cos(yaw), p.y + nav_center_x * math.sin(yaw), yaw


def nearest(times, values, stamp):
    if not times:
        return None
    idx = bisect.bisect_left(times, stamp)
    if idx <= 0:
        return values[0]
    if idx >= len(times):
        return values[-1]
    return values[idx - 1] if abs(times[idx - 1] - stamp) <= abs(times[idx] - stamp) else values[idx]


def to_frame(start, x, y):
    sx, sy, syaw = start
    dx = x - sx
    dy = y - sy
    c = math.cos(syaw)
    s = math.sin(syaw)
    return c * dx + s * dy, -s * dx + c * dy


def rel_pose(start, pose):
    x, y = to_frame(start, pose[0], pose[1])
    dyaw = math.degrees(math.atan2(math.sin(pose[2] - start[2]), math.cos(pose[2] - start[2])))
    return x, y, dyaw


def rel_pose_rad(start, pose):
    x, y = to_frame(start, pose[0], pose[1])
    dyaw = math.atan2(math.sin(pose[2] - start[2]), math.cos(pose[2] - start[2]))
    return x, y, dyaw


def signed_box_clearance(x, y, half_x, half_y):
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


def tape_points(perp_x, perp_y_min, perp_y_max, sample_step):
    n = max(1, int(math.ceil((perp_y_max - perp_y_min) / sample_step)))
    return [
        (perp_x, perp_y_min + (perp_y_max - perp_y_min) * i / n)
        for i in range(n + 1)
    ]


def first_contact(start_pose, odom_samples, action_start, args, padded):
    half_length = args.half_length + (args.padding if padded else 0.0)
    half_width = args.half_width + (args.padding if padded else 0.0)
    best = None
    first = None
    points = tape_points(args.perp_x, args.perp_y_min, args.perp_y_max, args.sample_step)

    for stamp, pose in odom_samples:
        if stamp < action_start:
            continue
        rx, ry, ryaw = rel_pose_rad(start_pose, pose)
        c = math.cos(ryaw)
        s = math.sin(ryaw)
        for px, py in points:
            dx = px - rx
            dy = py - ry
            robot_x = c * dx + s * dy
            robot_y = -s * dx + c * dy
            clearance = signed_box_clearance(robot_x, robot_y, half_length, half_width)
            sample = (clearance, stamp, rx, ry, math.degrees(ryaw), px, py)
            if best is None or clearance < best[0]:
                best = sample
            if first is None and clearance < 0.0:
                first = sample
    return first, best


def finite(value):
    return math.isfinite(float(value))


def valid_twist(twist):
    total = float(twist.total)
    if not finite(total) or total < 0.0:
        return False
    return all(finite(float(score.raw_score)) and float(score.raw_score) >= 0.0 for score in twist.scores)


def invalid_reason(twist):
    for score in twist.scores:
        raw = float(score.raw_score)
        if not finite(raw) or raw < 0.0:
            return score.name
    if not finite(float(twist.total)):
        return "nonfinite_total"
    if float(twist.total) < 0.0:
        return "negative_total"
    return "unknown"


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


def path_len(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def uuid_tuple(status):
    return tuple(int(v) for v in status.goal_info.goal_id.uuid)


def nonzero(vx, wz):
    return abs(vx) > 1e-4 or abs(wz) > 1e-4


def print_status_goals(label, store, bag_start):
    print(f"\n{label} status goals")
    if not store:
        print("  none")
        return
    for goal_id, events in store.items():
        states = [STATUS.get(st, str(st)) for _, st in events]
        print(
            f"  {list(goal_id)} first={events[0][0] - bag_start:.2f}s "
            f"last={events[-1][0] - bag_start:.2f}s final={states[-1]} states={states}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--perp-x", type=float, default=1.34)
    parser.add_argument("--perp-y-threshold", type=float, default=0.35)
    parser.add_argument("--perp-y-min", type=float, default=-0.13)
    parser.add_argument("--perp-y-max", type=float, default=0.50)
    parser.add_argument("--half-length", type=float, default=0.545)
    parser.add_argument("--half-width", type=float, default=0.41)
    parser.add_argument("--padding", type=float, default=0.03)
    parser.add_argument("--sample-step", type=float, default=0.005)
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
    if not raw or bag_start is None:
        raise SystemExit("empty bag or no selected topics")

    odom_t = []
    odom = []
    cmd = []
    cmd_nav = []
    evals = []
    statuses = {}
    follow_statuses = {}
    for topic, msg, stamp in raw:
        if topic == "/local_ekf/odom":
            odom_t.append(stamp)
            odom.append(nav_pose(msg))
        elif topic == "/cmd_vel":
            cmd.append((stamp, msg.linear.x, msg.angular.z))
        elif topic == "/cmd_vel_nav":
            cmd_nav.append((stamp, msg.linear.x, msg.angular.z))
        elif topic == "/evaluation":
            evals.append((stamp, msg))
        elif topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                statuses.setdefault(uuid_tuple(status), []).append((stamp, int(status.status)))
        elif topic == "/follow_path/_action/status":
            for status in msg.status_list:
                follow_statuses.setdefault(uuid_tuple(status), []).append((stamp, int(status.status)))

    print("Lidar-line test timeline")
    print(f"bag: {args.bag}")
    print(f"duration={raw[-1][2] - bag_start:.2f}s")
    print_status_goals("navigate", statuses, bag_start)
    print_status_goals("follow_path", follow_statuses, bag_start)

    action_start = None
    if statuses:
        exec_times = [stamp for events in statuses.values() for stamp, status in events if status == 2]
        if exec_times:
            action_start = min(exec_times)
    if action_start is None:
        plan_times = [stamp for topic, _, stamp in raw if topic == "/plan"]
        if plan_times:
            action_start = plan_times[0]
        elif odom_t:
            action_start = odom_t[0]
        else:
            action_start = bag_start
    start_pose = nearest(odom_t, odom, action_start)
    print(f"\naction_start_used={action_start - bag_start:.2f}s")

    if start_pose and odom:
        end_rel = rel_pose(start_pose, odom[-1])
        print(f"final odom from action_start: fwd={end_rel[0]:+.3f} left={end_rel[1]:+.3f} yaw={end_rel[2]:+.1f}deg")

    physical_contact = best_physical = None
    padded_contact = best_padded = None
    if start_pose and odom:
        odom_samples = list(zip(odom_t, odom))
        physical_contact, best_physical = first_contact(
            start_pose, odom_samples, action_start, args, padded=False)
        padded_contact, best_padded = first_contact(
            start_pose, odom_samples, action_start, args, padded=True)

    print("\nmeasured course contact")
    for label, contact, best in (
        ("physical", physical_contact, best_physical),
        ("padded", padded_contact, best_padded),
    ):
        if contact:
            print(
                f"  {label} first_contact={contact[1] - bag_start:.2f}s "
                f"action_t={contact[1] - action_start:.2f}s clearance={contact[0]:+.3f} "
                f"pose=({contact[2]:+.3f},{contact[3]:+.3f},{contact[4]:+.1f}deg) "
                f"tape=({contact[5]:+.3f},{contact[6]:+.3f})"
            )
        else:
            print(f"  {label} first_contact=none")
        if best:
            print(
                f"  {label} best_clearance={best[0]:+.3f} "
                f"at {best[1] - bag_start:.2f}s tape=({best[5]:+.3f},{best[6]:+.3f})"
            )

    nz_cmd = [(t, vx, wz) for t, vx, wz in cmd if nonzero(vx, wz)]
    nz_nav = [(t, vx, wz) for t, vx, wz in cmd_nav if nonzero(vx, wz)]
    print("\ncommands")
    if nz_cmd:
        print(f"  /cmd_vel nonzero first={nz_cmd[0][0] - bag_start:.2f}s last={nz_cmd[-1][0] - bag_start:.2f}s count={len(nz_cmd)}")
    else:
        print("  /cmd_vel nonzero none")
    if nz_nav:
        print(f"  /cmd_vel_nav nonzero first={nz_nav[0][0] - bag_start:.2f}s last={nz_nav[-1][0] - bag_start:.2f}s count={len(nz_nav)}")
    else:
        print("  /cmd_vel_nav nonzero none")

    first_perp_point = None
    first_any_line = None
    latest_line = None
    for topic, msg, stamp in raw:
        if topic != "/lidar_line_points" or stamp < action_start:
            continue
        pose = nearest(odom_t, odom, stamp)
        pts = []
        for point in msg.points:
            if msg.header.frame_id in ("odom", "map"):
                if start_pose:
                    pts.append(to_frame(start_pose, point.x, point.y))
            else:
                pts.append((point.x, point.y))
        if pts:
            first_any_line = first_any_line or (stamp, pts)
            latest_line = (stamp, pts)
        for x, y in pts:
            if 1.15 <= x <= 1.75 and y < args.perp_y_threshold:
                first_perp_point = first_perp_point or (stamp, x, y, len(pts))

    print("\nlidar line detections after action_start")
    if first_any_line:
        xs = [p[0] for p in first_any_line[1]]
        ys = [p[1] for p in first_any_line[1]]
        print(
            f"  first any={first_any_line[0] - bag_start:.2f}s n={len(first_any_line[1])} "
            f"x=[{min(xs):+.2f},{max(xs):+.2f}] y=[{min(ys):+.2f},{max(ys):+.2f}]"
        )
    if first_perp_point:
        print(
            f"  first likely perpendicular/rightward point={first_perp_point[0] - bag_start:.2f}s "
            f"point=({first_perp_point[1]:+.3f},{first_perp_point[2]:+.3f}) n={first_perp_point[3]}"
        )
        if physical_contact:
            delta = first_perp_point[0] - physical_contact[1]
            verdict = "before" if delta < 0.0 else "after"
            print(
                f"  first perpendicular detection was {abs(delta):.2f}s {verdict} "
                "physical footprint contact"
            )
    else:
        print("  first likely perpendicular/rightward point=none")
    if latest_line:
        xs = [p[0] for p in latest_line[1]]
        ys = [p[1] for p in latest_line[1]]
        print(
            f"  last any={latest_line[0] - bag_start:.2f}s n={len(latest_line[1])} "
            f"x=[{min(xs):+.2f},{max(xs):+.2f}] y=[{min(ys):+.2f},{max(ys):+.2f}]"
        )

    plans = []
    for topic, msg, stamp in raw:
        if topic != "/plan" or stamp < action_start:
            continue
        if not start_pose:
            continue
        pts = [to_frame(start_pose, pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if pts:
            plans.append((stamp, pts))

    print("\nplans after action_start")
    print(f"  count={len(plans)}")
    for index, (stamp, pts) in enumerate(plans[:5]):
        print(
            f"  plan[{index}] t={stamp - bag_start:.2f}s poses={len(pts)} len={path_len(pts):.2f} "
            f"y_at_x{args.perp_x:.2f}={y_at_x(pts, args.perp_x)} "
            f"y_range=[{min(y for _, y in pts):+.2f},{max(y for _, y in pts):+.2f}]"
        )
    if len(plans) > 5:
        stamp, pts = plans[-1]
        print(
            f"  last t={stamp - bag_start:.2f}s poses={len(pts)} len={path_len(pts):.2f} "
            f"y_at_x{args.perp_x:.2f}={y_at_x(pts, args.perp_x)} "
            f"y_range=[{min(y for _, y in pts):+.2f},{max(y for _, y in pts):+.2f}]"
        )

    print("\nDWB evaluation")
    if not evals:
        print("  none")
        return

    first_eval = evals[0][0]
    first_all_invalid = None
    first_valid_counts = []
    reasons = Counter()
    for stamp, msg in evals:
        valid = 0
        sample_reasons = Counter()
        for twist in msg.twists:
            if valid_twist(twist):
                valid += 1
            else:
                reason = invalid_reason(twist)
                reasons[reason] += 1
                sample_reasons[reason] += 1
        if len(first_valid_counts) < 8:
            first_valid_counts.append((stamp, valid, len(msg.twists), sample_reasons.most_common(2)))
        if valid == 0 and first_all_invalid is None:
            first_all_invalid = (stamp, sample_reasons.most_common(3))

    print(f"  first_eval={first_eval - bag_start:.2f}s eval_count={len(evals)}")
    for stamp, valid, total, sample_reasons in first_valid_counts:
        print(f"  eval t={stamp - bag_start:.2f}s valid={valid}/{total} reasons={sample_reasons}")
    if first_all_invalid:
        print(f"  first_all_invalid={first_all_invalid[0] - bag_start:.2f}s reasons={first_all_invalid[1]}")
    print(f"  invalid reasons total={reasons.most_common(5)}")


if __name__ == "__main__":
    main()
