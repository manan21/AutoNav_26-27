#!/usr/bin/env python3
"""Summarize PointCloud2 obstacle points relative to the nav_center footprint."""

from __future__ import annotations

import argparse
import math
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


def signed_box_clearance(x, y, half_x, half_y):
    dx = abs(x) - half_x
    dy = abs(y) - half_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return outside


def to_nav_center(frame_id, x, y, z, lidar_x):
    if frame_id == "lidar_footprint":
        # Static TF nav_center -> lidar_footprint is x=+0.435 m with a
        # 180-degree roll, so y/z are sign-flipped into nav_center.
        return x + lidar_x, -y, -z
    if frame_id == "nav_center":
        return x, y, z
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/scan_pca_filtered_points")
    parser.add_argument("--half-length", type=float, default=0.545)
    parser.add_argument("--half-width", type=float, default=0.41)
    parser.add_argument("--lidar-x", type=float, default=0.435)
    parser.add_argument("--stride", type=int, default=20)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        raise SystemExit(f"{args.topic} not found")
    msg_type = get_message(topic_types[args.topic])

    seen = 0
    analyzed = 0
    inside_counts = []
    nearest = None
    first_inside = None
    frame_counts = {}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic != args.topic:
            continue
        seen += 1
        if (seen - 1) % args.stride:
            continue
        msg = deserialize_message(data, msg_type)
        analyzed += 1
        frame_counts[msg.header.frame_id] = frame_counts.get(msg.header.frame_id, 0) + 1
        inside = 0
        sample_nearest = None
        sample_first_inside = None
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            converted = to_nav_center(msg.header.frame_id, point[0], point[1], point[2], args.lidar_x)
            if converted is None:
                continue
            x, y, z = converted
            clearance = signed_box_clearance(x, y, args.half_length, args.half_width)
            candidate = (clearance, stamp_ns / 1e9, x, y, z)
            if nearest is None or candidate[0] < nearest[0]:
                nearest = candidate
            if sample_nearest is None or candidate[0] < sample_nearest[0]:
                sample_nearest = candidate
            if clearance <= 0.0:
                inside += 1
                if sample_first_inside is None:
                    sample_first_inside = candidate
        inside_counts.append(inside)
        if inside and first_inside is None:
            first_inside = sample_first_inside

    print("PointCloud footprint analysis")
    print(f"bag: {args.bag}")
    print(f"topic: {args.topic}")
    print(f"samples analyzed: {analyzed} / {seen} (stride={args.stride})")
    print(f"frames: {frame_counts}")
    if inside_counts:
        print(
            "points inside footprint per sample: "
            f"min/median/max={min(inside_counts)}/"
            f"{statistics.median(inside_counts):.0f}/{max(inside_counts)}"
        )
    if first_inside:
        c, stamp, x, y, z = first_inside
        print(
            "first inside point: "
            f"clearance={c:+.3f} x={x:+.3f} y={y:+.3f} z={z:+.3f}"
        )
    if nearest:
        c, stamp, x, y, z = nearest
        print(
            "nearest point: "
            f"clearance={c:+.3f} x={x:+.3f} y={y:+.3f} z={z:+.3f}"
        )


if __name__ == "__main__":
    main()
