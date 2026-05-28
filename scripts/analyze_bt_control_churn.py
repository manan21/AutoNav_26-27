#!/usr/bin/env python3
"""Classify Nav2 BT/controller churn in a lidar-line simulation bag.

The main question this answers is whether FollowPath ABORTED statuses are
ordinary path-replacement churn from frequent replanning, or disruptive
controller/planner failures that stop the robot.
"""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import math
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/cmd_vel",
    "/cmd_vel_nav",
    "/navigate_to_pose/_action/status",
    "/follow_path/_action/status",
    "/compute_path_to_pose/_action/status",
    "/plan",
    "/rosout",
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


@dataclass(frozen=True)
class GoalSummary:
    goal_id: tuple[int, ...]
    first_stamp: float
    executing_stamp: float | None
    terminal_stamp: float | None
    final_state: int


@dataclass(frozen=True)
class AbortClassification:
    goal: GoalSummary
    replacement_like: bool
    disruptive: bool
    nearest_plan_delta: float | None
    next_goal_delta: float | None
    command_gap: float | None
    reason: str


def uuid_tuple(status) -> tuple[int, ...]:
    return tuple(int(value) for value in status.goal_info.goal_id.uuid)


def status_name(value: int | None) -> str:
    if value is None:
        return "none"
    return STATUS.get(value, str(value))


def nonzero(vx: float, wz: float) -> bool:
    return (
        math.isfinite(vx) and
        math.isfinite(wz) and
        (abs(vx) > 1e-4 or abs(wz) > 1e-4)
    )


def add_status(store: dict[tuple[int, ...], list[tuple[float, int]]],
               status,
               stamp: float) -> None:
    goal_id = uuid_tuple(status)
    state = int(status.status)
    events = store.setdefault(goal_id, [])
    if not events or events[-1][1] != state:
        events.append((stamp, state))


def bag_messages(bag_path: str, topics: set[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
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


def first_time(events: list[tuple[float, int]], states: set[int]) -> float | None:
    for stamp, state in events:
        if state in states:
            return stamp
    return None


def first_terminal(events: list[tuple[float, int]]) -> tuple[float | None, int | None]:
    for stamp, state in events:
        if state in TERMINAL:
            return stamp, state
    if not events:
        return None, None
    return None, events[-1][1]


def summarize_goals(
    store: dict[tuple[int, ...], list[tuple[float, int]]]
) -> list[GoalSummary]:
    goals = []
    for goal_id, events in store.items():
        if not events:
            continue
        terminal_stamp, final_state = first_terminal(events)
        goals.append(GoalSummary(
            goal_id=goal_id,
            first_stamp=events[0][0],
            executing_stamp=first_time(events, {2}),
            terminal_stamp=terminal_stamp,
            final_state=events[-1][1] if final_state is None else final_state,
        ))
    return sorted(goals, key=lambda goal: goal.first_stamp)


def latest_goal(goals: list[GoalSummary]) -> GoalSummary | None:
    if not goals:
        return None
    return max(goals, key=lambda goal: goal.first_stamp)


def count_states(goals: list[GoalSummary]) -> dict[int, int]:
    counts: dict[int, int] = {}
    for goal in goals:
        counts[goal.final_state] = counts.get(goal.final_state, 0) + 1
    return counts


def state_count_text(counts: dict[int, int]) -> str:
    if not counts:
        return "none"
    return ", ".join(
        f"{status_name(state)}={counts[state]}"
        for state in sorted(counts)
    )


def nearest_delta(times: list[float], stamp: float) -> float | None:
    if not times:
        return None
    idx = bisect.bisect_left(times, stamp)
    candidates = []
    if idx > 0:
        candidates.append(abs(stamp - times[idx - 1]))
    if idx < len(times):
        candidates.append(abs(times[idx] - stamp))
    return min(candidates) if candidates else None


def next_delta(times: list[float], stamp: float) -> float | None:
    idx = bisect.bisect_right(times, stamp)
    if idx >= len(times):
        return None
    return times[idx] - stamp


def command_gap_around(nonzero_times: list[float], stamp: float) -> float | None:
    if not nonzero_times:
        return None
    idx = bisect.bisect_left(nonzero_times, stamp)
    before = nonzero_times[idx - 1] if idx > 0 else None
    after = nonzero_times[idx] if idx < len(nonzero_times) else None
    if before is None or after is None:
        return None
    return after - before


def max_consecutive_gap(times: list[float]) -> tuple[float, tuple[float, float] | None]:
    if len(times) < 2:
        return 0.0, None
    best = 0.0
    best_pair = None
    for a, b in zip(times, times[1:]):
        gap = b - a
        if gap > best:
            best = gap
            best_pair = (a, b)
    return best, best_pair


def median_period(times: list[float]) -> float | None:
    if len(times) < 2:
        return None
    periods = [b - a for a, b in zip(times, times[1:]) if b > a]
    if not periods:
        return None
    return statistics.median(periods)


def max_consecutive_terminal_state(goals: list[GoalSummary], state: int) -> int:
    terminals = [
        goal
        for goal in goals
        if goal.terminal_stamp is not None
    ]
    terminals.sort(key=lambda goal: goal.terminal_stamp or goal.first_stamp)
    best = 0
    current = 0
    for goal in terminals:
        if goal.final_state == state:
            current += 1
            best = max(best, current)
        elif goal.final_state in TERMINAL:
            current = 0
    return best


def classify_follow_path_aborts(
    follow_goals: list[GoalSummary],
    plan_times: list[float],
    nonzero_times: list[float],
    replacement_window: float,
    command_gap_window: float,
) -> list[AbortClassification]:
    starts = [goal.first_stamp for goal in follow_goals]
    classifications = []
    for goal in follow_goals:
        if goal.final_state != 6 or goal.terminal_stamp is None:
            continue
        terminal = goal.terminal_stamp
        plan_delta = nearest_delta(plan_times, terminal)
        next_goal = next_delta(starts, goal.first_stamp)
        cmd_gap = command_gap_around(nonzero_times, terminal)
        near_plan = plan_delta is not None and plan_delta <= replacement_window
        near_next_goal = next_goal is not None and next_goal <= replacement_window
        command_stable = cmd_gap is None or cmd_gap <= command_gap_window
        replacement_like = (near_plan or near_next_goal) and command_stable
        disruptive = not replacement_like
        reasons = []
        if near_plan:
            reasons.append("near plan update")
        if near_next_goal:
            reasons.append("near next FollowPath goal")
        if not command_stable:
            reasons.append("command gap around abort")
        if not reasons:
            reasons.append("not near plan/goal replacement")
        classifications.append(AbortClassification(
            goal=goal,
            replacement_like=replacement_like,
            disruptive=disruptive,
            nearest_plan_delta=plan_delta,
            next_goal_delta=next_goal,
            command_gap=cmd_gap,
            reason=", ".join(reasons),
        ))
    return classifications


def fmt_delta(value: float | None) -> str:
    return "none" if value is None else f"{value:.3f}s"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--replacement-window", type=float, default=0.60)
    parser.add_argument("--command-gap-window", type=float, default=0.25)
    parser.add_argument("--warn-cmd-gap", type=float, default=0.50)
    parser.add_argument("--fail-cmd-gap", type=float, default=None)
    parser.add_argument("--fail-disruptive-follow-path-aborts", type=int, default=None)
    parser.add_argument("--fail-compute-aborts", type=int, default=None)
    parser.add_argument("--fail-consecutive-compute-aborts", type=int, default=None)
    args = parser.parse_args()

    navigate_status: dict[tuple[int, ...], list[tuple[float, int]]] = {}
    follow_status: dict[tuple[int, ...], list[tuple[float, int]]] = {}
    compute_status: dict[tuple[int, ...], list[tuple[float, int]]] = {}
    cmd_nav: list[tuple[float, float, float]] = []
    cmd_out: list[tuple[float, float, float]] = []
    plan_times: list[float] = []
    rosout_events: dict[str, list[tuple[float, str]]] = {
        "path_footprint_safe_rejects": [],
        "follow_path_action_server_aborts": [],
        "compute_path_action_server_aborts": [],
        "controller_goal_cancels": [],
        "recovery_waits": [],
        "passing_new_path": [],
        "missed_controller_rate": [],
        "no_valid_control": [],
        "progress_failures": [],
    }
    bag_start = None
    bag_end = None

    for topic, msg, stamp in bag_messages(args.bag, TOPICS):
        bag_start = stamp if bag_start is None else min(bag_start, stamp)
        bag_end = stamp if bag_end is None else max(bag_end, stamp)
        if topic == "/navigate_to_pose/_action/status":
            for status in msg.status_list:
                add_status(navigate_status, status, stamp)
        elif topic == "/follow_path/_action/status":
            for status in msg.status_list:
                add_status(follow_status, status, stamp)
        elif topic == "/compute_path_to_pose/_action/status":
            for status in msg.status_list:
                add_status(compute_status, status, stamp)
        elif topic == "/cmd_vel_nav":
            cmd_nav.append((stamp, float(msg.linear.x), float(msg.angular.z)))
        elif topic == "/cmd_vel":
            cmd_out.append((stamp, float(msg.linear.x), float(msg.angular.z)))
        elif topic == "/plan" and msg.poses:
            plan_times.append(stamp)
        elif topic == "/rosout":
            text = str(msg.msg)
            if "PathFootprintSafe: rejecting path" in text:
                rosout_events["path_footprint_safe_rejects"].append((stamp, text))
            if "[follow_path] [ActionServer] Aborting handle" in text:
                rosout_events["follow_path_action_server_aborts"].append((stamp, text))
            if "[compute_path_to_pose] [ActionServer] Aborting handle" in text:
                rosout_events["compute_path_action_server_aborts"].append((stamp, text))
            if "Goal was canceled. Stopping the robot" in text:
                rosout_events["controller_goal_cancels"].append((stamp, text))
            if "Running wait" in text:
                rosout_events["recovery_waits"].append((stamp, text))
            if "Passing new path to controller" in text:
                rosout_events["passing_new_path"].append((stamp, text))
            if "Failed to meet update rate" in text:
                rosout_events["missed_controller_rate"].append((stamp, text))
            if "No valid control" in text or "No valid trajectories" in text:
                rosout_events["no_valid_control"].append((stamp, text))
            if "Failed to make progress" in text:
                rosout_events["progress_failures"].append((stamp, text))

    if bag_start is None:
        raise SystemExit("empty bag or no selected topics")

    navigate_goals = summarize_goals(navigate_status)
    follow_goals = summarize_goals(follow_status)
    compute_goals = summarize_goals(compute_status)
    nav_goal = latest_goal(navigate_goals)
    action_start = (
        nav_goal.executing_stamp if nav_goal and nav_goal.executing_stamp is not None
        else nav_goal.first_stamp if nav_goal else bag_start
    )
    action_end = (
        nav_goal.terminal_stamp if nav_goal and nav_goal.terminal_stamp is not None
        else bag_end if bag_end is not None else action_start
    )

    window_cmd = [
        (stamp, vx, wz)
        for stamp, vx, wz in cmd_nav
        if action_start <= stamp <= action_end
    ]
    window_cmd_out = [
        (stamp, vx, wz)
        for stamp, vx, wz in cmd_out
        if action_start <= stamp <= action_end
    ]
    nonzero_times = [
        stamp
        for stamp, vx, wz in window_cmd
        if nonzero(vx, wz)
    ]
    nonzero_out_times = [
        stamp
        for stamp, vx, wz in window_cmd_out
        if nonzero(vx, wz)
    ]
    window_plans = [
        stamp
        for stamp in plan_times
        if action_start <= stamp <= action_end
    ]

    max_gap, max_gap_pair = max_consecutive_gap(nonzero_times)
    max_out_gap, max_out_gap_pair = max_consecutive_gap(nonzero_out_times)
    gaps_over_warn = [
        (a, b)
        for a, b in zip(nonzero_times, nonzero_times[1:])
        if b - a > args.warn_cmd_gap
    ]
    out_gaps_over_warn = [
        (a, b)
        for a, b in zip(nonzero_out_times, nonzero_out_times[1:])
        if b - a > args.warn_cmd_gap
    ]
    plan_period = median_period(window_plans)
    classifications = classify_follow_path_aborts(
        follow_goals=[
            goal for goal in follow_goals
            if action_start <= goal.first_stamp <= action_end
        ],
        plan_times=window_plans,
        nonzero_times=nonzero_times,
        replacement_window=args.replacement_window,
        command_gap_window=args.command_gap_window,
    )

    compute_in_window = [
        goal for goal in compute_goals
        if action_start <= goal.first_stamp <= action_end
    ]
    compute_aborts = sum(1 for goal in compute_in_window if goal.final_state == 6)
    consecutive_compute_aborts = max_consecutive_terminal_state(compute_in_window, 6)
    replacement_aborts = sum(1 for item in classifications if item.replacement_like)
    disruptive_aborts = sum(1 for item in classifications if item.disruptive)

    print("BT/control churn analysis")
    print(f"bag: {args.bag}")
    print(
        f"window: {action_start - bag_start:.2f}s..{action_end - bag_start:.2f}s "
        f"duration={action_end - action_start:.2f}s"
    )
    if nav_goal:
        print(
            "NavigateToPose: "
            f"final={status_name(nav_goal.final_state)} "
            f"goal={list(nav_goal.goal_id)}"
        )
    else:
        print("NavigateToPose: none")

    hz_text = "none" if plan_period is None else f"{1.0 / plan_period:.2f} Hz"
    print(
        f"plans: count={len(window_plans)} median_period={fmt_delta(plan_period)} "
        f"approx_rate={hz_text}"
    )

    print(
        "FollowPath goals: "
        f"count={len([g for g in follow_goals if action_start <= g.first_stamp <= action_end])} "
        f"states={state_count_text(count_states([g for g in follow_goals if action_start <= g.first_stamp <= action_end]))}"
    )
    print(
        "FollowPath abort classification: "
        f"total={len(classifications)} replacement_like={replacement_aborts} "
        f"disruptive={disruptive_aborts}"
    )
    for item in classifications[:5]:
        print(
            "  abort sample: "
            f"t={item.goal.terminal_stamp - bag_start:.2f}s "
            f"replacement_like={item.replacement_like} "
            f"plan_delta={fmt_delta(item.nearest_plan_delta)} "
            f"next_goal_delta={fmt_delta(item.next_goal_delta)} "
            f"cmd_gap={fmt_delta(item.command_gap)} "
            f"reason={item.reason}"
        )
    if len(classifications) > 5:
        print(f"  ... {len(classifications) - 5} additional FollowPath aborts")
    disruptive_items = [item for item in classifications if item.disruptive]
    for item in disruptive_items[:5]:
        print(
            "  disruptive abort: "
            f"t={item.goal.terminal_stamp - bag_start:.2f}s "
            f"plan_delta={fmt_delta(item.nearest_plan_delta)} "
            f"next_goal_delta={fmt_delta(item.next_goal_delta)} "
            f"cmd_gap={fmt_delta(item.command_gap)} "
            f"reason={item.reason}"
        )
    if len(disruptive_items) > 5:
        print(f"  ... {len(disruptive_items) - 5} additional disruptive aborts")

    print(
        "ComputePathToPose goals: "
        f"count={len(compute_in_window)} "
        f"states={state_count_text(count_states(compute_in_window))} "
        f"aborts={compute_aborts} "
        f"max_consecutive_aborts={consecutive_compute_aborts}"
    )

    print("commands:")
    if nonzero_times:
        pair_text = "none"
        if max_gap_pair is not None:
            pair_text = (
                f"{max_gap_pair[0] - bag_start:.2f}s.."
                f"{max_gap_pair[1] - bag_start:.2f}s"
            )
        print(
            f"  /cmd_vel_nav nonzero first={nonzero_times[0] - bag_start:.2f}s "
            f"last={nonzero_times[-1] - bag_start:.2f}s "
            f"count={len(nonzero_times)} max_gap={max_gap:.3f}s "
            f"max_gap_window={pair_text} gaps>{args.warn_cmd_gap:.2f}s={len(gaps_over_warn)}"
        )
    else:
        print("  /cmd_vel_nav nonzero none")
    if nonzero_out_times:
        pair_text = "none"
        if max_out_gap_pair is not None:
            pair_text = (
                f"{max_out_gap_pair[0] - bag_start:.2f}s.."
                f"{max_out_gap_pair[1] - bag_start:.2f}s"
            )
        print(
            f"  /cmd_vel nonzero first={nonzero_out_times[0] - bag_start:.2f}s "
            f"last={nonzero_out_times[-1] - bag_start:.2f}s "
            f"count={len(nonzero_out_times)} max_gap={max_out_gap:.3f}s "
            f"max_gap_window={pair_text} gaps>{args.warn_cmd_gap:.2f}s={len(out_gaps_over_warn)}"
        )
    else:
        print("  /cmd_vel nonzero none")

    if any(rosout_events.values()):
        print("rosout diagnostics:")
        for key, events in rosout_events.items():
            print(f"  {key}: {len(events)}")
            for stamp, text in events[:3]:
                print(f"    t={stamp - bag_start:.2f}s {text}")
            if len(events) > 3:
                print(f"    ... {len(events) - 3} additional events")
    else:
        print("rosout diagnostics: none recorded or no matching events")

    failures = []
    warnings = []
    if disruptive_aborts:
        warnings.append(f"FollowPath had {disruptive_aborts} disruptive-looking abort(s)")
    if compute_aborts:
        warnings.append(f"ComputePathToPose aborted {compute_aborts} time(s)")
    if gaps_over_warn:
        warnings.append(f"/cmd_vel_nav had {len(gaps_over_warn)} nonzero-command gap(s) > {args.warn_cmd_gap:.2f}s")
    if out_gaps_over_warn:
        warnings.append(f"/cmd_vel had {len(out_gaps_over_warn)} nonzero-command gap(s) > {args.warn_cmd_gap:.2f}s")

    if args.fail_disruptive_follow_path_aborts is not None and (
            disruptive_aborts > args.fail_disruptive_follow_path_aborts):
        failures.append(
            "disruptive FollowPath abort count "
            f"{disruptive_aborts} > {args.fail_disruptive_follow_path_aborts}")
    if args.fail_compute_aborts is not None and compute_aborts > args.fail_compute_aborts:
        failures.append(f"ComputePathToPose abort count {compute_aborts} > {args.fail_compute_aborts}")
    if (
        args.fail_consecutive_compute_aborts is not None and
        consecutive_compute_aborts > args.fail_consecutive_compute_aborts
    ):
        failures.append(
            "consecutive ComputePathToPose abort count "
            f"{consecutive_compute_aborts} > {args.fail_consecutive_compute_aborts}")
    if args.fail_cmd_gap is not None and max_out_gap > args.fail_cmd_gap:
        failures.append(f"/cmd_vel max nonzero gap {max_out_gap:.3f}s > {args.fail_cmd_gap:.3f}s")

    for warning in warnings:
        print(f"WARN: {warning}")

    if failures:
        print("\nFAIL: BT/control churn analysis")
        for failure in failures:
            print(f"  {failure}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
