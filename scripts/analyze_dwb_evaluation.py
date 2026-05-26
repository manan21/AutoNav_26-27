#!/usr/bin/env python3
"""Summarize DWB LocalPlanEvaluation messages from a ROS 2 bag."""

from __future__ import annotations

import argparse
import math
import statistics
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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


def sampled_topic_messages(bag_path, topic, stride=1, max_samples=0):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise SystemExit(f"{topic} not found in bag")
    msg_type = get_message(topic_types[topic])

    seen = 0
    yielded = 0
    while reader.has_next():
        msg_topic, data, stamp = reader.read_next()
        if msg_topic != topic:
            continue
        seen += 1
        if (seen - 1) % stride:
            continue
        yield deserialize_message(data, msg_type), stamp / 1e9
        yielded += 1
        if max_samples and yielded >= max_samples:
            break


def finite(value):
    return math.isfinite(float(value))


def invalid_reason(twist):
    for score in twist.scores:
        raw = float(score.raw_score)
        if not finite(raw) or raw < 0.0:
            return score.name
    total = float(twist.total)
    if not finite(total):
        return "nonfinite_total"
    if total < 0.0:
        return "negative_total"
    return "unknown"


def valid_twist(twist):
    total = float(twist.total)
    if not finite(total) or total < 0.0:
        return False
    return all(finite(float(score.raw_score)) and float(score.raw_score) >= 0.0 for score in twist.scores)


def top_contributors(twist, limit=5):
    entries = []
    for score in twist.scores:
        raw = float(score.raw_score)
        scale = float(score.scale)
        if not finite(raw):
            weighted = float("inf")
        else:
            weighted = raw * scale
        entries.append((abs(weighted), score.name, raw, scale, weighted))
    entries.sort(reverse=True)
    return entries[:limit]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/evaluation")
    parser.add_argument("--window", type=float, default=0.0, help="Only print all-invalid windows longer than this many seconds")
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Analyze every Nth evaluation sample to speed up large bags",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="Stop after this many analyzed evaluation samples; 0 means no limit",
    )
    args = parser.parse_args()
    if args.stride < 1:
        raise SystemExit("--stride must be >= 1")

    evaluations = []
    bag_start = None
    for msg, stamp in sampled_topic_messages(
        args.bag,
        args.topic,
        stride=args.stride,
        max_samples=args.max_samples,
    ):
        if bag_start is None:
            bag_start = stamp
        evaluations.append((stamp, msg))

    if not evaluations:
        raise SystemExit(f"no {args.topic} messages")

    bag_start = evaluations[0][0] if bag_start is None else bag_start
    invalid_reasons = Counter()
    valid_counts = []
    total_counts = []
    best_vx = []
    best_wz = []
    best_totals = []
    all_invalid_spans = []
    current_span = None
    first_all_invalid_samples = []
    best_contribs = defaultdict(list)

    for stamp, msg in evaluations:
        total = len(msg.twists)
        valid = 0
        sample_invalid_reasons = Counter()
        for twist in msg.twists:
            if valid_twist(twist):
                valid += 1
            else:
                reason = invalid_reason(twist)
                invalid_reasons[reason] += 1
                sample_invalid_reasons[reason] += 1

        total_counts.append(total)
        valid_counts.append(valid)

        if valid == 0:
            if current_span is None:
                current_span = [stamp, stamp]
            else:
                current_span[1] = stamp
            if len(first_all_invalid_samples) < 8:
                first_all_invalid_samples.append((stamp, total, sample_invalid_reasons.most_common(5)))
        elif current_span is not None:
            all_invalid_spans.append(tuple(current_span))
            current_span = None

        if 0 <= msg.best_index < len(msg.twists):
            best = msg.twists[msg.best_index]
            best_vx.append(float(best.traj.velocity.x))
            best_wz.append(float(best.traj.velocity.theta))
            best_totals.append(float(best.total))
            for _, name, raw, scale, weighted in top_contributors(best, limit=8):
                if finite(weighted):
                    best_contribs[name].append(weighted)

    if current_span is not None:
        all_invalid_spans.append(tuple(current_span))

    def rel(stamp):
        return stamp - bag_start

    print("DWB evaluation analysis")
    print(f"bag: {args.bag}")
    print(f"topic: {args.topic}")
    sample_text = f"samples analyzed: {len(evaluations)}"
    if args.stride != 1 or args.max_samples:
        sample_text += f" (stride={args.stride}, max_samples={args.max_samples or 'all'})"
    print(sample_text)
    print(f"time range: {rel(evaluations[0][0]):.2f}s to {rel(evaluations[-1][0]):.2f}s")
    print(
        "trajectories per sample: "
        f"min/median/max={min(total_counts)}/"
        f"{statistics.median(total_counts):.0f}/{max(total_counts)}"
    )
    print(
        "valid trajectories per sample: "
        f"min/median/max={min(valid_counts)}/"
        f"{statistics.median(valid_counts):.0f}/{max(valid_counts)}"
    )
    print(f"all-invalid samples: {sum(1 for count in valid_counts if count == 0)}")

    if best_vx:
        print(
            "best command: "
            f"vx min/median/max={min(best_vx):+.3f}/"
            f"{statistics.median(best_vx):+.3f}/{max(best_vx):+.3f} m/s, "
            f"wz min/median/max={min(best_wz):+.3f}/"
            f"{statistics.median(best_wz):+.3f}/{max(best_wz):+.3f} rad/s"
        )
        print(
            "best total score: "
            f"min/median/max={min(best_totals):+.3f}/"
            f"{statistics.median(best_totals):+.3f}/{max(best_totals):+.3f}"
        )

    print("\nInvalid trajectory reasons")
    for name, count in invalid_reasons.most_common(12):
        print(f"  {name}: {count}")

    if best_contribs:
        print("\nLargest weighted contributors in selected best trajectories")
        summaries = []
        for name, values in best_contribs.items():
            summaries.append((statistics.median(values), max(values), name, len(values)))
        for median, max_value, name, count in sorted(summaries, reverse=True)[:12]:
            print(f"  {name}: median={median:.3f} max={max_value:.3f} n={count}")

    if all_invalid_spans:
        print("\nAll-invalid spans")
        for start, end in all_invalid_spans:
            duration = end - start
            if duration < args.window:
                continue
            print(f"  {rel(start):.2f}s to {rel(end):.2f}s duration={duration:.2f}s")

    if first_all_invalid_samples:
        print("\nFirst all-invalid samples")
        for stamp, total, reasons in first_all_invalid_samples:
            reason_text = ", ".join(f"{name}={count}" for name, count in reasons)
            print(f"  t={rel(stamp):.2f}s total={total} {reason_text}")


if __name__ == "__main__":
    main()
