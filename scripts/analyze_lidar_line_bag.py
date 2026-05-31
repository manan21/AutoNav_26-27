#!/usr/bin/env python3
"""Summarize one lidar-line avoidance course rosbag.

Run inside the ROS 2 environment that has this workspace sourced:

    python3 scripts/analyze_lidar_line_bag.py /path/to/bag
"""

from __future__ import annotations

import argparse
import bisect
import math
import statistics
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


STATUS = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def nav_center_pose_from_odom(msg, nav_center_x):
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    return (
        p.x + nav_center_x * math.cos(yaw),
        p.y + nav_center_x * math.sin(yaw),
        yaw,
    )


def rel_pose(start, pose):
    sx, sy, syaw = start
    x, y, yaw = pose
    dx = x - sx
    dy = y - sy
    c = math.cos(syaw)
    s = math.sin(syaw)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        math.degrees(wrap_angle(yaw - syaw)),
    )


def uuid_tuple(goal_status):
    return tuple(int(v) for v in goal_status.goal_info.goal_id.uuid)


def nonzero_cmd(cmd):
    _, vx, wz = cmd
    return abs(vx) > 1e-4 or abs(wz) > 1e-4


def nearest_index(times, t):
    if not times:
        return None
    idx = bisect.bisect_left(times, t)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    before = idx - 1
    return before if abs(times[before] - t) <= abs(times[idx] - t) else idx


def path_length(poses):
    total = 0.0
    last = None
    for pose in poses:
        p = pose.pose.position
        if last is not None:
            total += math.hypot(p.x - last[0], p.y - last[1])
        last = (p.x, p.y)
    return total


def signed_box_clearance(x, y, half_x, half_y):
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


def summarize_status(topic, goals, start_t, end_t):
    final_counts = defaultdict(int)
    active = []
    for goal_id, events in goals.get(topic, {}).items():
        windowed = [(t, st) for t, st in events if start_t <= t <= end_t]
        if not windowed:
            continue
        final_counts[windowed[-1][1]] += 1
        first = windowed[0][0] - start_t
        last = windowed[-1][0] - start_t
        active.append((last, first, goal_id, windowed[-1][1], [st for _, st in windowed]))
    active.sort()
    counts = ", ".join(
        f"{STATUS.get(st, st)}={count}" for st, count in sorted(final_counts.items())
    )
    latest = []
    for last, first, goal_id, st, states in active[-5:]:
        latest.append(
            f"{first:5.1f}-{last:5.1f}s {STATUS.get(st, st)} "
            f"states={[STATUS.get(s, s) for s in states]}"
        )
    return counts or "none", latest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--half-length", type=float, default=0.545)
    parser.add_argument("--half-width", type=float, default=0.41)
    args = parser.parse_args()

    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        topic_metadata.name: topic_metadata.type
        for topic_metadata in reader.get_all_topics_and_types()
    }
    needed = {
        "/local_ekf/odom",
        "/cmd_vel",
        "/cmd_vel_nav",
        "/encoders",
        "/lidar_line_points",
        "/lidar_line_costmap",
        "/scan_pca_filtered_points",
        "/plan",
        "/local_plan",
        "/navigate_to_pose/_action/status",
        "/follow_path/_action/status",
        "/compute_path_to_pose/_action/status",
    }
    msg_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in needed
    }

    odom = []
    odom_child_frames = defaultdict(int)
    cmd = []
    cmd_nav = []
    enc = []
    line_points = []
    paths = defaultdict(list)
    goals = defaultdict(lambda: defaultdict(list))
    costmap_stats = []
    pca_count = 0
    pca_nonempty = 0
    pca_widths = []
    bag_start = None
    bag_end = None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        stamp = stamp_ns * 1e-9
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        bag_end = stamp if bag_end is None else max(bag_end, stamp)
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])

        if topic == "/local_ekf/odom":
            odom.append((stamp, nav_center_pose_from_odom(msg, args.nav_center_x)))
            odom_child_frames[msg.child_frame_id] += 1
        elif topic == "/cmd_vel":
            cmd.append((stamp, msg.linear.x, msg.angular.z))
        elif topic == "/cmd_vel_nav":
            cmd_nav.append((stamp, msg.linear.x, msg.angular.z))
        elif topic == "/encoders":
            enc.append((
                stamp,
                msg.left_motor_count,
                msg.right_motor_count,
                msg.left_motor_rpm,
                msg.right_motor_rpm,
            ))
        elif topic == "/lidar_line_points":
            line_points.append((
                stamp,
                msg.header.frame_id,
                [(point.x, point.y) for point in msg.points],
            ))
        elif topic == "/lidar_line_costmap":
            data_vals = msg.data
            pos = 0
            lethal = 0
            max_val = -1
            for val in data_vals:
                if val > 0:
                    pos += 1
                    if val >= 90:
                        lethal += 1
                    if val > max_val:
                        max_val = val
            costmap_stats.append((stamp, pos, lethal, max_val))
        elif topic == "/scan_pca_filtered_points":
            pca_count += 1
            width = int(msg.width) * int(msg.height)
            pca_widths.append(width)
            if width > 0:
                pca_nonempty += 1
        elif topic in ("/plan", "/local_plan"):
            paths[topic].append((stamp, len(msg.poses), path_length(msg.poses)))
        elif topic.endswith("/_action/status"):
            for status in msg.status_list:
                goals[topic][uuid_tuple(status)].append((stamp, int(status.status)))

    if bag_start is None:
        raise SystemExit("empty bag")
    if not odom:
        raise SystemExit("bag has no /local_ekf/odom samples")

    odom_times = [t for t, _ in odom]

    nav_goal = None
    nav_topic = "/navigate_to_pose/_action/status"
    for goal_id, events in goals.get(nav_topic, {}).items():
        if any(st == 2 for _, st in events):
            candidate = (events[-1][0], goal_id, events)
            if nav_goal is None or candidate[0] > nav_goal[0]:
                nav_goal = candidate

    if nav_goal:
        _, goal_id, nav_events = nav_goal
        exec_times = [t for t, st in nav_events if st == 2]
        terminal_times = [t for t, st in nav_events if st in (4, 5, 6)]
        start_t = exec_times[0] if exec_times else nav_events[0][0]
        end_t = terminal_times[-1] if terminal_times else nav_events[-1][0]
        result_status = nav_events[-1][1]
    else:
        all_cmds = [entry for entry in cmd_nav if nonzero_cmd(entry)]
        if not all_cmds:
            all_cmds = [entry for entry in cmd if nonzero_cmd(entry)]
        start_t = all_cmds[0][0] if all_cmds else odom[0][0]
        end_t = all_cmds[-1][0] if all_cmds else odom[-1][0]
        goal_id = None
        result_status = None

    if end_t < start_t:
        end_t = start_t

    start_idx = nearest_index(odom_times, start_t)
    end_idx = nearest_index(odom_times, end_t)
    start_pose = odom[start_idx][1]
    end_pose = odom[end_idx][1]
    final_rel = rel_pose(start_pose, end_pose)

    odom_window = [(t, pose) for t, pose in odom if start_t <= t <= end_t]
    rel_samples = [rel_pose(start_pose, pose) for _, pose in odom_window]
    fwd_vals = [sample[0] for sample in rel_samples]
    left_vals = [sample[1] for sample in rel_samples]
    yaw_vals = [sample[2] for sample in rel_samples]

    cmd_window = [entry for entry in cmd if start_t <= entry[0] <= end_t]
    cmd_nav_window = [entry for entry in cmd_nav if start_t <= entry[0] <= end_t]
    nonzero_cmds = [entry for entry in cmd_window if nonzero_cmd(entry)]
    first_cmd_t = nonzero_cmds[0][0] if nonzero_cmds else None
    last_cmd_t = nonzero_cmds[-1][0] if nonzero_cmds else None
    cmd_v = [abs(vx) for _, vx, _ in nonzero_cmds]
    cmd_w = [abs(wz) for _, _, wz in nonzero_cmds]

    bins = []
    small_turn_bins = 0
    small_turn_motion_bins = 0
    stalled_bins = 0
    total_nonzero_bins = 0
    t0 = math.floor(start_t)
    t1 = math.ceil(end_t)
    for sec in range(int(t0), int(t1)):
        a = float(sec)
        b = float(sec + 1)
        bin_cmds = [entry for entry in cmd_window if a <= entry[0] < b and nonzero_cmd(entry)]
        if not bin_cmds:
            continue
        bin_odom = [(t, pose) for t, pose in odom_window if a <= t < b]
        if len(bin_odom) < 2:
            continue
        total_nonzero_bins += 1
        max_v = max(abs(vx) for _, vx, _ in bin_cmds)
        max_w = max(abs(wz) for _, _, wz in bin_cmds)
        rel_a = rel_pose(bin_odom[0][1], bin_odom[-1][1])
        dist = math.hypot(rel_a[0], rel_a[1])
        yaw_delta = abs(rel_a[2])
        bins.append((a - start_t, max_v, max_w, dist, yaw_delta))
        if max_v < 0.03 and 0.02 <= max_w <= 0.10:
            small_turn_bins += 1
            if dist >= 0.005 or yaw_delta >= 0.5:
                small_turn_motion_bins += 1
        if (max_v >= 0.01 or max_w >= 0.03) and dist < 0.005 and yaw_delta < 0.5:
            stalled_bins += 1

    enc_window = [entry for entry in enc if start_t <= entry[0] <= end_t]
    enc_delta = None
    enc_rpm_nonzero = 0
    if len(enc_window) >= 2:
        first = enc_window[0]
        last = enc_window[-1]
        enc_delta = (last[1] - first[1], last[2] - first[2])
        enc_rpm_nonzero = sum(1 for e in enc_window if e[3] != 0 or e[4] != 0)

    line_clearance = None
    line_nonempty = []
    line_first_summary = None
    inside_count = 0
    for stamp, frame, points in line_points:
        if not (start_t <= stamp <= end_t) or not points:
            continue
        idx = nearest_index(odom_times, stamp)
        if idx is None:
            continue
        pose = odom[idx][1]
        rx, ry, ryaw = pose
        c = math.cos(ryaw)
        s = math.sin(ryaw)
        rel_pts = []
        for x, y in points:
            if frame == "odom":
                dx = x - rx
                dy = y - ry
                robot_x = c * dx + s * dy
                robot_y = -s * dx + c * dy
            else:
                robot_x = x
                robot_y = y
            clearance = signed_box_clearance(
                robot_x,
                robot_y,
                args.half_length,
                args.half_width,
            )
            if clearance < 0.0:
                inside_count += 1
            candidate = (clearance, stamp, robot_x, robot_y, frame)
            if line_clearance is None or candidate[0] < line_clearance[0]:
                line_clearance = candidate
            rel_pts.append((robot_x, robot_y))
        xs = [p[0] for p in rel_pts]
        ys = [p[1] for p in rel_pts]
        line_nonempty.append((stamp, len(points)))
        if line_first_summary is None:
            line_first_summary = (
                stamp,
                len(points),
                min(xs),
                max(xs),
                min(ys),
                max(ys),
            )

    local_plan_window = [entry for entry in paths["/local_plan"] if start_t <= entry[0] <= end_t]
    global_plan_window = [entry for entry in paths["/plan"] if start_t <= entry[0] <= end_t]
    zero_local = sum(1 for _, n, _ in local_plan_window if n == 0)
    nonzero_local = len(local_plan_window) - zero_local
    local_lengths = [length for _, n, length in local_plan_window if n > 0]
    global_lengths = [length for _, n, length in global_plan_window if n > 0]

    costmap_window = [entry for entry in costmap_stats if start_t <= entry[0] <= end_t]
    lethal_entries = [entry for entry in costmap_window if entry[2] > 0]
    positive_entries = [entry for entry in costmap_window if entry[1] > 0]
    max_lethal = max((entry[2] for entry in costmap_window), default=0)
    max_cost = max((entry[3] for entry in costmap_window), default=-1)

    pca_summary = "not recorded"
    if pca_count:
        pca_summary = (
            f"{pca_nonempty}/{pca_count} nonempty, "
            f"width min/median/max="
            f"{min(pca_widths)}/{statistics.median(pca_widths):.0f}/{max(pca_widths)}"
        )

    print("Lidar line avoidance bag analysis")
    print(f"bag: {args.bag}")
    print(f"duration: {bag_end - bag_start:.1f}s")
    print(f"selected window: {start_t - bag_start:.1f}s to {end_t - bag_start:.1f}s "
          f"({end_t - start_t:.1f}s)")
    if goal_id is not None:
        print(f"navigate goal: {list(goal_id)} final={STATUS.get(result_status, result_status)}")
    else:
        print("navigate goal: not found; window selected from nonzero cmd_vel")
    if odom_child_frames:
        frame_text = ", ".join(f"{k or '<empty>'}={v}" for k, v in odom_child_frames.items())
        print(f"/local_ekf/odom child_frame_id counts: {frame_text}")

    print("")
    print("Trajectory from nav_center")
    print(
        f"final rel: fwd={final_rel[0]:+.3f} m, left={final_rel[1]:+.3f} m, "
        f"yaw={final_rel[2]:+.1f} deg"
    )
    if rel_samples:
        print(
            f"range: fwd=[{min(fwd_vals):+.3f},{max(fwd_vals):+.3f}] m, "
            f"left=[{min(left_vals):+.3f},{max(left_vals):+.3f}] m, "
            f"yaw=[{min(yaw_vals):+.1f},{max(yaw_vals):+.1f}] deg"
        )

    print("")
    print("Commands and drivetrain response")
    if first_cmd_t is None:
        print("nonzero /cmd_vel: none")
    else:
        print(
            f"first/last nonzero /cmd_vel: {first_cmd_t - start_t:.1f}s / "
            f"{last_cmd_t - start_t:.1f}s after window start"
        )
        print(
            f"/cmd_vel nonzero samples={len(nonzero_cmds)}, "
            f"|vx| max/median={max(cmd_v):.3f}/{statistics.median(cmd_v):.3f} m/s, "
            f"|wz| max/median={max(cmd_w):.3f}/{statistics.median(cmd_w):.3f} rad/s"
        )
    if enc_delta is not None:
        print(
            f"encoder delta: left={enc_delta[0]}, right={enc_delta[1]}, "
            f"nonzero RPM samples={enc_rpm_nonzero}/{len(enc_window)}"
        )
    print(
        f"1s nonzero-command bins={total_nonzero_bins}, "
        f"no-measurable-motion bins={stalled_bins}, "
        f"small-turn bins with motion={small_turn_motion_bins}/{small_turn_bins}"
    )

    print("")
    print("Line detection and costmap")
    if line_first_summary:
        stamp, count, min_x, max_x, min_y, max_y = line_first_summary
        print(
            f"first nonempty /lidar_line_points at {stamp - start_t:.1f}s: "
            f"n={count}, robot-frame x=[{min_x:+.2f},{max_x:+.2f}] m, "
            f"y=[{min_y:+.2f},{max_y:+.2f}] m"
        )
        print(
            f"last nonempty /lidar_line_points at "
            f"{line_nonempty[-1][0] - start_t:.1f}s"
        )
    else:
        print("nonempty /lidar_line_points: none inside selected window")
    if line_clearance:
        clearance, stamp, x, y, frame = line_clearance
        overlap = "YES" if clearance < 0.0 else "no"
        print(
            f"closest detected line point to nav_center footprint: "
            f"clearance={clearance:+.3f} m at {stamp - start_t:.1f}s, "
            f"point=({x:+.3f},{y:+.3f}) m, overlap={overlap}"
        )
        print(f"detected line points inside footprint box: {inside_count}")
    if costmap_window:
        if lethal_entries:
            lethal_delay = ""
            if line_nonempty:
                lethal_delay = (
                    f", last_lethal_after_last_detection="
                    f"{lethal_entries[-1][0] - line_nonempty[-1][0]:+.1f}s"
                )
            print(
                f"/lidar_line_costmap lethal cells: max={max_lethal}, "
                f"first={lethal_entries[0][0] - start_t:.1f}s, "
                f"last={lethal_entries[-1][0] - start_t:.1f}s"
                f"{lethal_delay}"
            )
        else:
            print("/lidar_line_costmap lethal cells: none")
        if positive_entries:
            positive_delay = ""
            if line_nonempty:
                positive_delay = (
                    f", last_positive_after_last_detection="
                    f"{positive_entries[-1][0] - line_nonempty[-1][0]:+.1f}s"
                )
            print(
                f"/lidar_line_costmap positive cells: first={positive_entries[0][0] - start_t:.1f}s, "
                f"last={positive_entries[-1][0] - start_t:.1f}s, max_cost={max_cost}"
                f"{positive_delay}"
            )

    print("")
    print("Plans and action status")
    if global_plan_window:
        print(
            f"/plan samples={len(global_plan_window)}, nonzero={sum(1 for _, n, _ in global_plan_window if n > 0)}, "
            f"path length median={statistics.median(global_lengths):.2f} m" if global_lengths else
            f"/plan samples={len(global_plan_window)}, no nonzero plans"
        )
    if local_plan_window:
        local_text = (
            f", path length median={statistics.median(local_lengths):.2f} m"
            if local_lengths else ""
        )
        print(
            f"/local_plan samples={len(local_plan_window)}, nonzero={nonzero_local}, "
            f"zero={zero_local}{local_text}"
        )
    for topic in (
        "/navigate_to_pose/_action/status",
        "/follow_path/_action/status",
        "/compute_path_to_pose/_action/status",
    ):
        counts, latest = summarize_status(topic, goals, start_t, end_t)
        print(f"{topic}: final counts in window: {counts}")
        for line in latest:
            print(f"  {line}")

    print("")
    print(f"PCA filtered point cloud: {pca_summary}")


if __name__ == "__main__":
    main()
