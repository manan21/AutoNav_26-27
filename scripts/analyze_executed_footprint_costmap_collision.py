#!/usr/bin/env python3
"""Check the executed odom footprint against raw global costmap lethal cells."""

from __future__ import annotations

import argparse
import math

from analyze_global_plan_costmap_collision import (
    angle_diff,
    bag_messages,
    costmap_cells_to_start,
    fmt_float,
    min_footprint_clearance,
    nearest_pose,
    pose_from_odom,
    preceding_entry,
    to_start_frame,
)


TOPICS = {
    "/local_ekf/odom",
    "/cmd_vel_nav",
    "/navigate_to_pose/_action/status",
    "/global_costmap/costmap_raw",
}

TERMINAL = {4, 5, 6}


def pose_to_start(start, pose):
    sx, sy = to_start_frame(start, pose[0], pose[1])
    return sx, sy, angle_diff(pose[2], start[2])


def nonzero(vx, wz):
    return math.isfinite(vx) and math.isfinite(wz) and (abs(vx) > 1e-4 or abs(wz) > 1e-4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--nav-center-x", type=float, default=0.225)
    parser.add_argument("--half-length", type=float, default=0.595)
    parser.add_argument("--half-width", type=float, default=0.46)
    parser.add_argument("--lethal-threshold", type=int, default=254)
    parser.add_argument("--unknown-value", type=int, default=255)
    parser.add_argument(
        "--clearance-margin",
        type=float,
        default=0.0,
        help="Minimum acceptable signed footprint clearance when --fail-on-overlap is set.",
    )
    parser.add_argument(
        "--fail-on-overlap",
        action="store_true",
        help="Exit nonzero if the executed footprint overlaps lethal raw global cells.",
    )
    parser.add_argument(
        "--require-costmap",
        action="store_true",
        help="Exit nonzero if no global costmap sample can be checked.",
    )
    args = parser.parse_args()

    odom_times = []
    odom_poses = []
    cmd_nav = []
    nav_events = []
    costmaps = []
    bag_start = None

    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        if topic == "/local_ekf/odom":
            odom_times.append(stamp)
            odom_poses.append(pose_from_odom(msg, args.nav_center_x))
        elif topic == "/cmd_vel_nav":
            cmd_nav.append((stamp, float(msg.linear.x), float(msg.angular.z)))
        elif topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                nav_events.append((stamp, int(status.status)))
        elif topic == "/global_costmap/costmap_raw":
            costmaps.append((stamp, msg))

    if bag_start is None:
        raise SystemExit("empty bag")
    if not odom_poses:
        raise SystemExit("no /local_ekf/odom samples")

    exec_times = [stamp for stamp, state in nav_events if state == 2]
    terminal_times = [stamp for stamp, state in nav_events if state in TERMINAL]
    action_start = min(exec_times) if exec_times else odom_times[0]
    action_end = min((stamp for stamp in terminal_times if stamp >= action_start), default=odom_times[-1])
    start_pose = nearest_pose(odom_times, odom_poses, action_start)
    if start_pose is None:
        raise SystemExit("no odom pose for action start")

    inspected = 0
    overlap_count = 0
    best = None
    cell_cache = {}
    nonzero_cmd = [
        (stamp, vx, wz)
        for stamp, vx, wz in cmd_nav
        if action_start <= stamp <= action_end and nonzero(vx, wz)
    ]

    for stamp, pose in zip(odom_times, odom_poses):
        if stamp < action_start or stamp > action_end:
            continue
        grid_entry = preceding_entry(costmaps, stamp)
        if grid_entry is None:
            continue
        grid_stamp, grid_msg = grid_entry
        cells = cell_cache.get(grid_stamp)
        if cells is None:
            cells = costmap_cells_to_start(
                grid_msg,
                args.lethal_threshold,
                args.unknown_value,
                start_pose,
                None,
            )
            cell_cache[grid_stamp] = cells
        if not cells:
            continue

        footprint_pose = pose_to_start(start_pose, pose)
        clearance = min_footprint_clearance(
            [footprint_pose],
            cells,
            args.half_length,
            args.half_width,
        )
        if clearance is None:
            continue
        inspected += 1
        sample = (
            clearance[0],
            stamp,
            grid_stamp,
            footprint_pose,
            clearance[1:],
            len(cells),
        )
        if best is None or sample[0] < best[0]:
            best = sample
        if clearance[0] <= args.clearance_margin:
            overlap_count += 1

    print("Executed footprint raw-costmap analysis")
    print(f"bag: {args.bag}")
    print(
        f"action_window={action_start - bag_start:.2f}s..{action_end - bag_start:.2f}s "
        f"odom_samples={len(odom_times)} costmap_samples={len(costmaps)} inspected={inspected}"
    )
    print(
        f"footprint half_length={args.half_length:.3f} half_width={args.half_width:.3f} "
        f"lethal_threshold={args.lethal_threshold}"
    )
    if nonzero_cmd:
        print(
            f"/cmd_vel_nav nonzero first={nonzero_cmd[0][0] - bag_start:.2f}s "
            f"last={nonzero_cmd[-1][0] - bag_start:.2f}s count={len(nonzero_cmd)}"
        )
    else:
        print("/cmd_vel_nav nonzero none")

    if best is None:
        print("nearest executed lethal-cell clearance: none")
    else:
        clearance, stamp, grid_stamp, pose, details, cell_count = best
        _, rel_x, rel_y, cost = details
        print(
            f"nearest executed lethal-cell clearance: {fmt_float(clearance)} "
            f"at t={stamp - bag_start:.2f}s costmap_age={stamp - grid_stamp:+.2f}s "
            f"pose=({pose[0]:+.3f},{pose[1]:+.3f},{math.degrees(pose[2]):+.1f}deg) "
            f"cell_rel=({rel_x:+.3f},{rel_y:+.3f}) cost={cost} lethal_cells={cell_count}"
        )
    print(f"executed footprint overlap samples: {overlap_count}")

    failures = []
    if args.require_costmap and inspected == 0:
        failures.append("no odom samples had a preceding nonempty /global_costmap/costmap_raw")
    if args.fail_on_overlap and overlap_count > 0:
        failures.append(
            f"{overlap_count} executed odom sample(s) had lethal clearance "
            f"<= margin {args.clearance_margin:+.3f}"
        )
    if failures:
        print("\nFAIL: executed footprint raw-costmap collision")
        for failure in failures:
            print(f"  {failure}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
