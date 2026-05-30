#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def _stamp_s(stamp: Any) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _cost_counts(msg: Any) -> dict[str, int]:
    values = [int(v) for v in msg.data]
    return {
        "max": max(values, default=0),
        "ge_1": sum(1 for v in values if v >= 1),
        "ge_100": sum(1 for v in values if v >= 100),
        "ge_254": sum(1 for v in values if v >= 254),
    }


def analyze_bag(bag_dir: Path) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    wanted = {
        "/clock",
        "/odom",
        "/igvc_sim/ground_truth_odom",
        "/cmd_vel_gazebo",
        "/igvc_sim/dynamics_state",
        "/igvc_sim/dynamics_calibration",
        "/goal_pose",
        "/goal_update",
        "/nav_goal",
        "/plan",
        "/cloud_all_fields_fullframe",
        "/scan_fullframe",
        "/scan_pca_filtered_points",
        "/scan_pca_filtered",
        "/scan_pca_filtered_clear",
        "/zed/zed_node/rgb/color/rect/camera_info",
        "/zed/zed_node/depth/depth_info",
        "/line_points",
        "/line_costmap",
        "/lidar_line_points",
        "/lidar_line_costmap",
        "/local_costmap/costmap_raw",
        "/global_costmap/costmap_raw",
        "/igvc_sim/score",
    }
    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in wanted
        if topic in topic_types
    }

    first_ts: int | None = None
    clock_max = 0.0
    header_samples: dict[str, list[dict[str, Any]]] = {}
    goals: list[dict[str, Any]] = []
    goal_updates: list[dict[str, Any]] = []
    nav_goals: list[dict[str, Any]] = []
    plans: list[dict[str, Any]] = []
    odom: list[dict[str, float]] = []
    costmaps: dict[str, list[dict[str, Any]]] = {}
    detector_counts: dict[str, list[dict[str, Any]]] = {}
    scores: list[dict[str, Any]] = []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in msg_types:
            continue
        if first_ts is None:
            first_ts = ts
        rel_s = (ts - first_ts) / 1e9
        msg = deserialize_message(data, msg_types[topic])

        if topic == "/clock":
            clock_max = max(clock_max, _stamp_s(msg.clock))
            continue

        if hasattr(msg, "header"):
            header_samples.setdefault(topic, [])
            if len(header_samples[topic]) < 5:
                header_samples[topic].append({
                    "rel_s": rel_s,
                    "stamp_s": _stamp_s(msg.header.stamp),
                    "frame": msg.header.frame_id,
                })

        if topic == "/odom":
            odom.append({
                "rel_s": rel_s,
                "stamp_s": _stamp_s(msg.header.stamp),
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "v": float(msg.twist.twist.linear.x),
            })
        elif topic == "/goal_pose":
            goals.append({
                "rel_s": rel_s,
                "stamp_s": _stamp_s(msg.header.stamp),
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
            })
        elif topic == "/goal_update":
            goal_updates.append({
                "rel_s": rel_s,
                "stamp_s": _stamp_s(msg.header.stamp),
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
            })
        elif topic == "/nav_goal":
            nav_goals.append({
                "rel_s": rel_s,
                "stamp_s": _stamp_s(msg.header.stamp),
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
            })
        elif topic == "/plan":
            if msg.poses:
                end = msg.poses[-1].pose.position
                start = msg.poses[0].pose.position
                plans.append({
                    "rel_s": rel_s,
                    "stamp_s": _stamp_s(msg.header.stamp),
                    "poses": len(msg.poses),
                    "start": [float(start.x), float(start.y)],
                    "end": [float(end.x), float(end.y)],
                })
        elif topic in ("/line_points", "/lidar_line_points"):
            detector_counts.setdefault(topic, []).append({
                "rel_s": rel_s,
                "points": len(msg.points),
            })
        elif topic.endswith("costmap_raw") or topic in (
            "/line_costmap",
            "/lidar_line_costmap",
        ):
            costmaps.setdefault(topic, []).append({
                "rel_s": rel_s,
                **_cost_counts(msg),
            })
        elif topic == "/igvc_sim/score":
            try:
                score = json.loads(msg.data)
            except json.JSONDecodeError:
                score = {"raw": msg.data}
            score["rel_s"] = rel_s
            scores.append(score)

    detector_topics = [
        "/scan_pca_filtered_points",
        "/scan_pca_filtered",
        "/scan_pca_filtered_clear",
        "/line_points",
        "/lidar_line_points",
    ]
    wall_stamped = []
    if 0.0 < clock_max < 1_000_000.0:
        for topic in detector_topics:
            for sample in header_samples.get(topic, []):
                if sample["stamp_s"] > 1_000_000.0:
                    wall_stamped.append(topic)
                    break

    stale_path_warnings = []
    for goal in goals:
        goal_xy = (goal["x"], goal["y"])
        candidates = [
            plan for plan in plans
            if plan["rel_s"] >= goal["rel_s"]
            and plan["rel_s"] <= goal["rel_s"] + 2.0
        ]
        matching = [
            plan for plan in candidates
            if plan["stamp_s"] + 0.10 >= goal["stamp_s"]
            and _dist(tuple(plan["end"]), goal_xy) <= 0.75
        ]
        if not candidates:
            previous = [
                plan for plan in plans
                if plan["rel_s"] < goal["rel_s"]
                and plan["rel_s"] >= goal["rel_s"] - 1.0
            ]
            if previous:
                stale_path_warnings.append({
                    "goal_rel_s": goal["rel_s"],
                    "goal": [goal["x"], goal["y"]],
                    "reason": "no plan published in 2s after goal",
                    "previous_plan_rel_s": previous[-1]["rel_s"],
                    "previous_plan_end": previous[-1]["end"],
                    "previous_plan_stamp_s": previous[-1]["stamp_s"],
                    "goal_stamp_s": goal["stamp_s"],
                })
        elif not matching:
            first = candidates[0]
            stale_path_warnings.append({
                "goal_rel_s": goal["rel_s"],
                "goal": [goal["x"], goal["y"]],
                "reason": "first post-goal plan did not match goal",
                "first_plan_rel_s": first["rel_s"],
                "first_plan_end": first["end"],
                "first_plan_stamp_s": first["stamp_s"],
                "goal_stamp_s": goal["stamp_s"],
            })

    summary: dict[str, Any] = {
        "bag": str(bag_dir),
        "clock_max_s": clock_max,
        "header_samples": header_samples,
        "wall_stamped_detector_topics": sorted(set(wall_stamped)),
        "goal_pose_count": len(goals),
        "goal_update_count": len(goal_updates),
        "nav_goal_count": len(nav_goals),
        "plan_count": len(plans),
        "stale_path_warnings": stale_path_warnings,
        "final_score": scores[-1] if scores else None,
        "odom_final": odom[-1] if odom else None,
        "odom_max_x": max((sample["x"] for sample in odom), default=None),
        "detector_point_ranges": {},
        "costmap_ranges": {},
    }

    for topic, samples in detector_counts.items():
        summary["detector_point_ranges"][topic] = {
            "samples": len(samples),
            "max_points": max((s["points"] for s in samples), default=0),
            "last_points": samples[-1]["points"] if samples else 0,
        }

    for topic, samples in costmaps.items():
        summary["costmap_ranges"][topic] = {
            "samples": len(samples),
            "max_value": max((s["max"] for s in samples), default=0),
            "last_max_value": samples[-1]["max"] if samples else 0,
            "max_ge_1": max((s["ge_1"] for s in samples), default=0),
            "last_ge_1": samples[-1]["ge_1"] if samples else 0,
            "max_ge_100": max((s["ge_100"] for s in samples), default=0),
            "last_ge_100": samples[-1]["ge_100"] if samples else 0,
            "max_ge_254": max((s["ge_254"] for s in samples), default=0),
            "last_ge_254": samples[-1]["ge_254"] if samples else 0,
        }
    return summary


def analyze_log(log_path: Path | None) -> dict[str, Any]:
    if log_path is None or not log_path.exists():
        return {"message_filter_drops": None}
    text = log_path.read_text(encoding="utf-8", errors="replace")
    return {
        "message_filter_drops": text.count("Message Filter dropping"),
        "path_goal_consistent_timeouts": text.count("PathGoalConsistent: stale path"),
        "blocking_stop_failures": text.count("blocking_stop_over_60s"),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_or_bag", help="Run directory or rosbag directory")
    parser.add_argument("--runner-log", default="", help="Optional runner.log path")
    parser.add_argument("--json-out", default="", help="Optional JSON output path")
    args = parser.parse_args(argv)

    run_or_bag = Path(args.run_or_bag).expanduser()
    bag_dir = run_or_bag / "bag" if (run_or_bag / "bag").exists() else run_or_bag
    log_path = Path(args.runner_log).expanduser() if args.runner_log else None
    if log_path is None and (run_or_bag / "runner.log").exists():
        log_path = run_or_bag / "runner.log"

    summary = analyze_bag(bag_dir)
    summary["log"] = analyze_log(log_path)

    print("IGVC run analysis")
    print(f"  bag: {summary['bag']}")
    print(f"  final_score: {summary['final_score']}")
    print(f"  odom_final: {summary['odom_final']}")
    print(f"  goals: /goal_pose={summary['goal_pose_count']} /goal_update={summary['goal_update_count']} /nav_goal={summary['nav_goal_count']}")
    print(f"  plans: {summary['plan_count']} stale_path_warnings={len(summary['stale_path_warnings'])}")
    print(f"  wall_stamped_detector_topics: {summary['wall_stamped_detector_topics']}")
    print(f"  log: {summary['log']}")
    for topic, ranges in sorted(summary["costmap_ranges"].items()):
        print(f"  {topic}: {ranges}")

    if args.json_out:
        Path(args.json_out).expanduser().write_text(
            json.dumps(summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
