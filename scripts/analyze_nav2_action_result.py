#!/usr/bin/env python3
"""Summarize and optionally enforce Nav2 action results for a ROS 2 bag."""

from __future__ import annotations

import argparse
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/cmd_vel_nav",
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

TERMINAL = {4, 5, 6}


def uuid_tuple(status):
    return tuple(int(v) for v in status.goal_info.goal_id.uuid)


def status_name(value):
    return STATUS.get(value, str(value))


def nonzero(vx, wz):
    return math.isfinite(vx) and math.isfinite(wz) and (abs(vx) > 1e-4 or abs(wz) > 1e-4)


def add_status(store, status, stamp):
    goal_id = uuid_tuple(status)
    state = int(status.status)
    events = store.setdefault(goal_id, [])
    if not events or events[-1][1] != state:
        events.append((stamp, state))


def bag_messages(bag_path, topics):
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


def latest_goal(store):
    if not store:
        return None, []
    return max(store.items(), key=lambda item: item[1][0][0])


def first_time(events, states):
    for stamp, state in events:
        if state in states:
            return stamp
    return None


def final_state(events):
    return events[-1][1] if events else None


def print_goals(label, store, bag_start):
    print(f"\n{label} goals")
    if not store:
        print("  none")
        return
    for goal_id, events in sorted(store.items(), key=lambda item: item[1][0][0]):
        states = [status_name(state) for _, state in events]
        print(
            f"  {list(goal_id)} first={events[0][0] - bag_start:.2f}s "
            f"last={events[-1][0] - bag_start:.2f}s final={states[-1]} states={states}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument(
        "--require-succeeded",
        action="store_true",
        help="Exit nonzero unless the latest NavigateToPose goal reaches SUCCEEDED.",
    )
    parser.add_argument(
        "--require-cmd-vel-nav",
        action="store_true",
        help="Exit nonzero unless /cmd_vel_nav has a nonzero sample during navigation.",
    )
    parser.add_argument(
        "--warn-follow-path-aborts",
        action="store_true",
        help="Print a warning when FollowPath reports ABORTED, but do not fail on it.",
    )
    args = parser.parse_args()

    navigate = {}
    follow_path = {}
    cmd_nav = []
    bag_start = None
    bag_end = None

    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        bag_end = stamp if bag_end is None else max(bag_end, stamp)
        if topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                add_status(navigate, status, stamp)
        elif topic == "/follow_path/_action/status":
            for status in msg.status_list:
                add_status(follow_path, status, stamp)
        elif topic == "/cmd_vel_nav":
            cmd_nav.append((stamp, float(msg.linear.x), float(msg.angular.z)))

    if bag_start is None:
        raise SystemExit("empty bag or no selected topics")

    print("Nav2 action result analysis")
    print(f"bag: {args.bag}")
    print_goals("NavigateToPose", navigate, bag_start)
    print_goals("FollowPath", follow_path, bag_start)

    nav_goal_id, nav_events = latest_goal(navigate)
    action_start = first_time(nav_events, {2}) if nav_events else None
    action_end = first_time(nav_events, TERMINAL) if nav_events else None
    if action_start is None:
        action_start = nav_events[0][0] if nav_events else bag_start
    if action_end is None:
        action_end = bag_end if bag_end is not None else action_start

    nz_cmd = [
        (stamp, vx, wz)
        for stamp, vx, wz in cmd_nav
        if action_start <= stamp <= action_end and nonzero(vx, wz)
    ]
    print("\ncommands")
    if nz_cmd:
        print(
            f"  /cmd_vel_nav nonzero first={nz_cmd[0][0] - bag_start:.2f}s "
            f"last={nz_cmd[-1][0] - bag_start:.2f}s count={len(nz_cmd)}"
        )
    else:
        print("  /cmd_vel_nav nonzero none")

    follow_aborts = sum(
        1
        for events in follow_path.values()
        for _, state in events
        if state == 6
    )
    if args.warn_follow_path_aborts and follow_aborts:
        print(f"\nWARN: FollowPath reported ABORTED {follow_aborts} time(s)")

    failures = []
    if args.require_succeeded:
        if not nav_events:
            failures.append("no NavigateToPose status goals")
        elif final_state(nav_events) != 4:
            goal_text = "unknown" if nav_goal_id is None else list(nav_goal_id)
            failures.append(
                f"latest NavigateToPose goal {goal_text} final state "
                f"{status_name(final_state(nav_events))}, expected SUCCEEDED"
            )
    if args.require_cmd_vel_nav and not nz_cmd:
        failures.append("no nonzero /cmd_vel_nav during latest NavigateToPose window")

    if failures:
        print("\nFAIL: Nav2 action result")
        for failure in failures:
            print(f"  {failure}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
