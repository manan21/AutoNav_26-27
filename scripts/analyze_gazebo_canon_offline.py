#!/usr/bin/env python3
"""Offline real-bag analysis for Gazebo canonicalization.

This script intentionally avoids rosbag2_py so it can run in the local ROS22
Lima VM that sees the bags and repo but lacks ROS 2's Python bag bindings.
It uses the pure-Python `rosbags` reader and registers the small AutoNav custom
message set needed for these bags.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
from typing import Any

try:
    import yaml
except Exception as exc:  # pragma: no cover
    raise SystemExit("PyYAML is required: %s" % exc) from exc

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None

RUNS = [
    {
        "name": "arc_ladder_1",
        "category": "dynamics",
        "bag_dir": "/Users/cole/autonav_bags/jetson_practice_course_20260529_211047/arc_ladder_1",
        "video": "/Users/cole/Downloads/arc_ladder_1.MOV",
    },
    {
        "name": "in_place_yaw_ladder_1",
        "category": "dynamics",
        "bag_dir": "/Users/cole/autonav_bags/jetson_practice_course_20260529_211047/in_place_yaw_ladder_1",
        "video": "/Users/cole/Downloads/in_place_yaw_ladder_1.MOV",
    },
    {
        "name": "straight_distance_drift_low_2",
        "category": "dynamics",
        "bag_dir": "/Users/cole/autonav_bags/jetson_practice_course_20260529_211047/straight_distance_drift_low_2",
        "video": "/Users/cole/Downloads/straight_distance_drift_low_2.MOV",
        "physical_measurement": {
            "forward_m": 9.3726,
            "right_drift_m": 1.5288,
            "right_drift_policy": "slope-contaminated; diagnostic only",
        },
    },
    {
        "name": "manual_course_rerun_5",
        "category": "full_perception",
        "bag_dir": "/Users/cole/autonav_bag_backups/manual_course_rerun_5",
        "video": "/Users/cole/Downloads/manual_course_rerun_5.MOV",
    },
    {
        "name": "manual_course_rerun_7",
        "category": "full_perception",
        "bag_dir": "/Users/cole/autonav_bag_backups/manual_course_rerun_7",
        "video": "/Users/cole/Downloads/manual_course_rerun_7.MOV",
    },
    {
        "name": "camera_line_detection_rerun",
        "category": "full_perception",
        "bag_dir": "/Users/cole/autonav_bag_backups/camera_line_detection_rerun",
        "video": "/Users/cole/Downloads/camera_line_detection_rerun.MOV",
    },
    {
        "name": "camera_line_detection_rerun_2feet_stationary",
        "category": "full_perception",
        "bag_dir": "/Users/cole/autonav_bag_backups/camera_line_detection_rerun_2feet_stationary",
        "video": "/Users/cole/Downloads/camera_line_detection_rerun_two_feet_stationary.MOV",
    },
]

VIDEO_FALLBACKS = {
    "arc_ladder_1.MOV": {"duration_s": 88.035, "width": 1920, "height": 1080},
    "in_place_yaw_ladder_1.MOV": {
        "duration_s": 81.802,
        "width": 1920,
        "height": 1080,
    },
    "straight_distance_drift_low_2.MOV": {
        "duration_s": 56.867,
        "width": 1920,
        "height": 1080,
    },
    "manual_course_rerun_5.MOV": {
        "duration_s": 195.237,
        "width": 1920,
        "height": 1080,
    },
    "manual_course_rerun_7.MOV": {
        "duration_s": 148.103,
        "width": 1920,
        "height": 1080,
    },
    "camera_line_detection_rerun.MOV": {
        "duration_s": 122.768,
        "width": 1920,
        "height": 1080,
    },
    "camera_line_detection_rerun_two_feet_stationary.MOV": {
        "duration_s": 59.6,
        "width": 1920,
        "height": 1080,
    },
}

SELECTED_TOPICS = {
    "/tf",
    "/tf_static",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/odom",
    "/local_ekf/odom",
    "/global_ekf/odom",
    "/encoders",
    "/motor_speed",
    "/autonomous_mode",
    "/joy",
    "/sick_scansegment_xd/imu",
    "/sick_scansegment_xd/imu_inflated",
    "/scan_fullframe",
    "/scan_pca_filtered",
    "/scan_pca_filtered_clear",
    "/scan_pca_filtered_points",
    "/terrain/grade_map",
    "/pca/surface_normal",
    "/zed/zed_node/rgb/color/rect/image",
    "/zed/zed_node/depth/depth_registered",
    "/line_detection/line_pixels",
    "/line_detection/diagnostics",
    "/line_detection/debug/raw",
    "/line_detection/debug/mask",
    "/line_detection/debug/overlay",
    "/line_points",
    "/line_costmap",
    "/lines_pointcloud",
    "/lidar_line_points",
    "/lidar_line_costmap",
    "/lidar_line_detection/diagnostics",
    "/lidar_line_detection/debug/points",
    "/map",
    "/map_padded",
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/plan",
    "/unsmoothed_plan",
    "/local_plan",
    "/trajectories",
    "/transformed_global_plan",
    "/navigate_to_pose/_action/status",
    "/follow_path/_action/status",
    "/compute_path_to_pose/_action/status",
}

IMAGE_TOPICS = {
    "/zed/zed_node/rgb/color/rect/image",
    "/zed/zed_node/depth/depth_registered",
    "/line_detection/debug/raw",
    "/line_detection/debug/mask",
    "/line_detection/debug/overlay",
}

ODOM_TOPICS = {"/odom", "/local_ekf/odom", "/global_ekf/odom"}
COSTMAP_TOPICS = {
    "/line_costmap",
    "/lidar_line_costmap",
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/map",
    "/map_padded",
    "/terrain/grade_map",
}
PLAN_TOPICS = {"/plan", "/unsmoothed_plan", "/local_plan", "/transformed_global_plan"}
TF_TOPICS = {"/tf", "/tf_static"}
SCAN_TOPICS = {"/scan_fullframe", "/scan_pca_filtered", "/scan_pca_filtered_clear"}
POINTCLOUD_TOPICS = {
    "/scan_pca_filtered_points",
    "/lines_pointcloud",
    "/lidar_line_detection/debug/points",
}


class NumericStats:
    def __init__(self) -> None:
        self.count = 0
        self.min_v: float | None = None
        self.max_v: float | None = None
        self.sum_v = 0.0

    def add(self, value: float | int | None) -> None:
        if value is None:
            return
        value_f = float(value)
        if math.isnan(value_f) or math.isinf(value_f):
            return
        self.count += 1
        self.sum_v += value_f
        self.min_v = value_f if self.min_v is None else min(self.min_v, value_f)
        self.max_v = value_f if self.max_v is None else max(self.max_v, value_f)

    def as_dict(self, digits: int = 4) -> dict[str, Any]:
        if self.count == 0:
            return {"count": 0}
        return {
            "count": self.count,
            "min": round(self.min_v or 0.0, digits),
            "max": round(self.max_v or 0.0, digits),
            "mean": round(self.sum_v / self.count, digits),
        }


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}


def read_run_metadata(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if ":" not in line or line.startswith("  "):
            continue
        key, value = line.split(":", 1)
        out[key.strip()] = value.strip()
    return out


def read_topic_list(path: Path) -> list[str]:
    if not path.is_file():
        return []
    topics = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = raw.strip()
        if stripped and not stripped.startswith("#"):
            topics.append(stripped)
    return topics


def read_metadata(path: Path) -> dict[str, Any]:
    data = load_yaml(path)
    info = data.get("rosbag2_bagfile_information", {})
    topics: dict[str, dict[str, Any]] = {}
    for item in info.get("topics_with_message_count", []) or []:
        meta = item.get("topic_metadata", {})
        name = meta.get("name")
        if not name:
            continue
        topics[str(name)] = {
            "type": str(meta.get("type", "")),
            "message_count": int(item.get("message_count", 0)),
        }
    return {
        "present": path.is_file(),
        "duration_s": round(
            float(info.get("duration", {}).get("nanoseconds", 0)) / 1e9, 3
        ),
        "starting_time_ns": int(
            info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)
        ),
        "message_count": int(info.get("message_count", 0)),
        "topics": topics,
    }


def video_info(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False, "path": str(path)}
    info: dict[str, Any] = {
        "present": True,
        "path": str(path),
        "size_mb": round(path.stat().st_size / (1024 * 1024), 2),
    }
    ffprobe = shutil.which("ffprobe")
    if ffprobe:
        cmd = [
            ffprobe,
            "-v",
            "error",
            "-show_entries",
            "format=duration:stream=width,height,avg_frame_rate",
            "-of",
            "json",
            str(path),
        ]
        try:
            raw = subprocess.check_output(cmd, text=True, timeout=10)
            parsed = json.loads(raw)
            if parsed.get("format", {}).get("duration"):
                info["duration_s"] = round(float(parsed["format"]["duration"]), 3)
            streams = parsed.get("streams") or []
            if streams:
                stream = streams[0]
                info["width"] = stream.get("width")
                info["height"] = stream.get("height")
                info["avg_frame_rate"] = stream.get("avg_frame_rate")
        except Exception as exc:
            info["ffprobe_error"] = str(exc)
    fallback = VIDEO_FALLBACKS.get(path.name)
    if fallback:
        for key, value in fallback.items():
            info.setdefault(key, value)
        if "duration_s" in fallback and "duration_s" not in info:
            info["duration_source"] = "macos_mdls_fallback"
    return info


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def add_top(top: list[dict[str, Any]], item: dict[str, Any], key: str, limit: int = 5) -> None:
    top.append(item)
    top.sort(key=lambda x: x.get(key, 0.0), reverse=True)
    del top[limit:]


def stats_dict(stats: dict[str, NumericStats]) -> dict[str, Any]:
    return {key: value.as_dict() for key, value in sorted(stats.items())}


def register_custom_types(repo: Path):
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    msg_root = repo / "isaac_ros-dev/src/autonav_interfaces/msg"
    for name in ("LinePoints", "Encoders"):
        path = msg_root / f"{name}.msg"
        if path.is_file():
            typestore.register(
                get_types_from_msg(
                    path.read_text(encoding="utf-8"),
                    f"autonav_interfaces/msg/{name}",
                )
            )
    return typestore


def summarize_odom(samples: dict[str, list[dict[str, float]]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for topic, values in samples.items():
        if len(values) < 2:
            out[topic] = {"count": len(values)}
            continue
        start = values[0]
        end = values[-1]
        dx = end["x"] - start["x"]
        dy = end["y"] - start["y"]
        c = math.cos(start["yaw"])
        s = math.sin(start["yaw"])
        forward = c * dx + s * dy
        lateral = -s * dx + c * dy
        points = [(v["x"], v["y"]) for v in values]
        out[topic] = {
            "count": len(values),
            "duration_s": round(end["t"] - start["t"], 3),
            "path_length_m": round(path_length(points), 4),
            "euclidean_delta_m": round(math.hypot(dx, dy), 4),
            "start_frame_forward_m": round(forward, 4),
            "start_frame_lateral_m": round(lateral, 4),
            "yaw_delta_deg": round(math.degrees(angle_diff(end["yaw"], start["yaw"])), 3),
            "mean_abs_linear_mps": round(
                statistics.fmean(abs(v["linear_x"]) for v in values), 4
            ),
            "mean_abs_angular_radps": round(
                statistics.fmean(abs(v["angular_z"]) for v in values), 4
            ),
        }
    return out


def summarize_cmd(samples: dict[str, list[dict[str, float]]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for topic, values in samples.items():
        nonzero = [
            v for v in values
            if abs(v["linear_x"]) > 1e-4 or abs(v["angular_z"]) > 1e-4
        ]
        out[topic] = {
            "count": len(values),
            "nonzero_count": len(nonzero),
        }
        if values:
            out[topic].update({
                "max_abs_linear_mps": round(max(abs(v["linear_x"]) for v in values), 4),
                "max_abs_angular_radps": round(max(abs(v["angular_z"]) for v in values), 4),
            })
        if nonzero:
            out[topic].update({
                "first_nonzero_s": round(nonzero[0]["t"], 3),
                "last_nonzero_s": round(nonzero[-1]["t"], 3),
                "mean_abs_linear_nonzero_mps": round(
                    statistics.fmean(abs(v["linear_x"]) for v in nonzero), 4
                ),
                "mean_abs_angular_nonzero_radps": round(
                    statistics.fmean(abs(v["angular_z"]) for v in nonzero), 4
                ),
            })
    return out


def image_luma(msg: Any) -> np.ndarray | None:
    if np is None:
        return None
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if data.size == 0:
        return None
    enc = str(msg.encoding).lower()
    if enc in ("mono8", "8uc1"):
        return data
    if enc in ("rgb8", "bgr8") and data.size >= int(msg.width) * int(msg.height) * 3:
        arr = data[: int(msg.width) * int(msg.height) * 3].reshape((-1, 3))
        return arr.astype(np.float32).mean(axis=1)
    if enc in ("rgba8", "bgra8") and data.size >= int(msg.width) * int(msg.height) * 4:
        arr = data[: int(msg.width) * int(msg.height) * 4].reshape((-1, 4))
        return arr[:, :3].astype(np.float32).mean(axis=1)
    return None


def analyze_run(run: dict[str, Any], repo: Path, typestore: Any, max_image_samples: int) -> dict[str, Any]:
    from rosbags.rosbag2 import Reader

    run_dir = Path(run["bag_dir"])
    bag_path = run_dir / "bag"
    metadata = read_metadata(bag_path / "metadata.yaml")
    run_metadata = read_run_metadata(run_dir / "run_metadata.txt")
    expected_topics = read_topic_list(run_dir / "topics.txt")
    missing_at_start = read_topic_list(run_dir / "missing_topics_at_start.txt")
    topics_meta = metadata.get("topics", {})
    zero_count_expected = [
        topic for topic in expected_topics
        if int(topics_meta.get(topic, {}).get("message_count", 0)) == 0
    ]

    result: dict[str, Any] = {
        "name": run["name"],
        "category": run["category"],
        "bag_dir": str(run_dir),
        "video": video_info(Path(run["video"])),
        "run_metadata": run_metadata,
        "physical_measurement": run.get("physical_measurement", {}),
        "bag_metadata": {
            "present": metadata["present"],
            "duration_s": metadata["duration_s"],
            "message_count": metadata["message_count"],
            "topic_count": len(topics_meta),
        },
        "topic_health": {
            "expected_topic_count": len(expected_topics),
            "missing_at_start": missing_at_start,
            "zero_count_expected_topics": zero_count_expected,
        },
        "decode_errors": Counter(),
    }
    if not bag_path.is_dir():
        result["error"] = "bag directory missing"
        return result

    start_ns = int(metadata.get("starting_time_ns") or 0)
    if start_ns <= 0:
        start_ns = None

    odom_samples: dict[str, list[dict[str, float]]] = defaultdict(list)
    cmd_samples: dict[str, list[dict[str, float]]] = defaultdict(list)
    encoder_summary: dict[str, Any] = {"count": 0}
    encoder_first: Any | None = None
    encoder_last: Any | None = None
    line_diag_stats: dict[str, NumericStats] = defaultdict(NumericStats)
    line_diag_reasons: Counter[str] = Counter()
    line_diag_samples: list[dict[str, Any]] = []
    lidar_diag_reasons: Counter[str] = Counter()
    line_pixels = {
        "messages": 0,
        "pixel_count": NumericStats(),
        "first_nonzero_s": None,
        "max_pixel_count": 0,
    }
    line_points = {
        "messages": 0,
        "point_count": NumericStats(),
        "first_nonzero_s": None,
        "max_point_count": 0,
        "frames": Counter(),
    }
    lidar_line_points = {
        "messages": 0,
        "point_count": NumericStats(),
        "first_nonzero_s": None,
        "max_point_count": 0,
        "frames": Counter(),
    }
    costmaps: dict[str, dict[str, Any]] = {
        topic: {
            "messages": 0,
            "nonzero_cells": NumericStats(),
            "hard_cells": NumericStats(),
            "first_nonzero_s": None,
            "max_value": None,
        }
        for topic in COSTMAP_TOPICS
    }
    scans: dict[str, dict[str, Any]] = {
        topic: {
            "messages": 0,
            "finite_ranges": NumericStats(),
            "close_ranges_lt_1m": NumericStats(),
            "min_range_m": NumericStats(),
            "closest_events": [],
        }
        for topic in SCAN_TOPICS
    }
    pointclouds: dict[str, dict[str, Any]] = {
        topic: {"messages": 0, "points": NumericStats(), "first_nonzero_s": None}
        for topic in POINTCLOUD_TOPICS
    }
    images: dict[str, dict[str, Any]] = {
        topic: {
            "messages": 0,
            "sampled": 0,
            "encoding": Counter(),
            "frame_ids": Counter(),
            "width": None,
            "height": None,
            "luma_mean": NumericStats(),
            "luma_max": NumericStats(),
            "bright_ratio_ge_220": NumericStats(),
            "nonzero_ratio": NumericStats(),
            "top_frame_diffs": [],
        }
        for topic in IMAGE_TOPICS
    }
    previous_luma: dict[str, np.ndarray] = {}
    plans: dict[str, dict[str, Any]] = {
        topic: {"messages": 0, "poses": NumericStats(), "length_m": NumericStats()}
        for topic in PLAN_TOPICS
    }
    tf_summary: dict[str, dict[str, Any]] = {
        topic: {
            "messages": 0,
            "transform_count": 0,
            "pairs": Counter(),
            "child_frames": Counter(),
            "parent_frames": Counter(),
            "first_bag_s": None,
            "last_bag_s": None,
            "first_stamp_s": None,
            "last_stamp_s": None,
        }
        for topic in TF_TOPICS
    }

    with Reader(bag_path) as reader:
        connections = [
            connection for connection in reader.connections
            if connection.topic in SELECTED_TOPICS
        ]
        topic_types = {connection.topic: connection.msgtype for connection in reader.connections}
        result["topic_types"] = {
            topic: topic_types[topic]
            for topic in sorted(topic_types)
            if topic in SELECTED_TOPICS
        }

        image_seen: Counter[str] = Counter()
        for connection, timestamp, raw in reader.messages(connections=connections):
            topic = connection.topic
            if start_ns is None:
                start_ns = timestamp
            rel_t = (timestamp - start_ns) / 1e9

            if topic in IMAGE_TOPICS:
                image_seen[topic] += 1
                if images[topic]["sampled"] >= max_image_samples:
                    images[topic]["messages"] += 1
                    continue

            try:
                msg = typestore.deserialize_cdr(raw, connection.msgtype)
            except Exception as exc:
                result["decode_errors"][f"{topic}:{type(exc).__name__}"] += 1
                continue

            if topic in ("/cmd_vel", "/cmd_vel_nav"):
                cmd_samples[topic].append({
                    "t": rel_t,
                    "linear_x": float(msg.linear.x),
                    "angular_z": float(msg.angular.z),
                })
            elif topic in ODOM_TOPICS:
                odom_samples[topic].append({
                    "t": rel_t,
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                    "linear_x": float(msg.twist.twist.linear.x),
                    "angular_z": float(msg.twist.twist.angular.z),
                })
            elif topic == "/encoders":
                encoder_summary["count"] += 1
                encoder_first = encoder_first or msg
                encoder_last = msg
            elif topic == "/line_detection/diagnostics":
                try:
                    diag = json.loads(str(msg.data))
                except Exception:
                    diag = {"reason": str(msg.data)}
                reason = str(diag.get("reason", ""))
                if reason:
                    line_diag_reasons[reason] += 1
                for key, value in diag.items():
                    if isinstance(value, (int, float)):
                        line_diag_stats[key].add(value)
                if len(line_diag_samples) < 5:
                    line_diag_samples.append({"t_s": round(rel_t, 3), "data": diag})
            elif topic == "/lidar_line_detection/diagnostics":
                try:
                    diag = json.loads(str(msg.data))
                    reason = str(diag.get("reason", ""))
                except Exception:
                    reason = str(msg.data)[:120]
                if reason:
                    lidar_diag_reasons[reason] += 1
            elif topic == "/line_detection/line_pixels":
                line_pixels["messages"] += 1
                pixel_count = max(0, (len(msg.data) - 2) // 2)
                line_pixels["pixel_count"].add(pixel_count)
                line_pixels["max_pixel_count"] = max(line_pixels["max_pixel_count"], pixel_count)
                if pixel_count > 0 and line_pixels["first_nonzero_s"] is None:
                    line_pixels["first_nonzero_s"] = round(rel_t, 3)
            elif topic == "/line_points":
                count = len(msg.points)
                line_points["messages"] += 1
                line_points["point_count"].add(count)
                line_points["max_point_count"] = max(line_points["max_point_count"], count)
                line_points["frames"][str(msg.header.frame_id)] += 1
                if count > 0 and line_points["first_nonzero_s"] is None:
                    line_points["first_nonzero_s"] = round(rel_t, 3)
            elif topic == "/lidar_line_points":
                count = len(msg.points)
                lidar_line_points["messages"] += 1
                lidar_line_points["point_count"].add(count)
                lidar_line_points["max_point_count"] = max(
                    lidar_line_points["max_point_count"], count)
                lidar_line_points["frames"][str(msg.header.frame_id)] += 1
                if count > 0 and lidar_line_points["first_nonzero_s"] is None:
                    lidar_line_points["first_nonzero_s"] = round(rel_t, 3)
            elif topic in COSTMAP_TOPICS:
                data = list(msg.data)
                nonzero = sum(1 for value in data if int(value) not in (0, -1))
                hard = sum(1 for value in data if int(value) >= 99)
                max_value = max((int(value) for value in data), default=None)
                cm = costmaps[topic]
                cm["messages"] += 1
                cm["nonzero_cells"].add(nonzero)
                cm["hard_cells"].add(hard)
                if max_value is not None:
                    cm["max_value"] = max_value if cm["max_value"] is None else max(cm["max_value"], max_value)
                if nonzero > 0 and cm["first_nonzero_s"] is None:
                    cm["first_nonzero_s"] = round(rel_t, 3)
            elif topic in SCAN_TOPICS:
                ranges = [float(value) for value in msg.ranges]
                finite = [value for value in ranges if math.isfinite(value)]
                scan = scans[topic]
                scan["messages"] += 1
                scan["finite_ranges"].add(len(finite))
                scan["close_ranges_lt_1m"].add(sum(1 for value in finite if value < 1.0))
                if finite:
                    min_range = min(finite)
                    scan["min_range_m"].add(min_range)
                    add_top(
                        scan["closest_events"],
                        {"t_s": round(rel_t, 3), "min_range_m": round(min_range, 4), "finite_ranges": len(finite)},
                        key="min_range_m",
                        limit=5,
                    )
                    scan["closest_events"].sort(key=lambda item: item["min_range_m"])
                    del scan["closest_events"][5:]
            elif topic in POINTCLOUD_TOPICS:
                point_count = int(msg.width) * int(msg.height)
                pc = pointclouds[topic]
                pc["messages"] += 1
                pc["points"].add(point_count)
                if point_count > 0 and pc["first_nonzero_s"] is None:
                    pc["first_nonzero_s"] = round(rel_t, 3)
            elif topic in IMAGE_TOPICS:
                item = images[topic]
                item["messages"] += 1
                item["sampled"] += 1
                item["encoding"][str(msg.encoding)] += 1
                item["frame_ids"][str(msg.header.frame_id)] += 1
                item["width"] = int(msg.width)
                item["height"] = int(msg.height)
                luma = image_luma(msg)
                if luma is not None and luma.size:
                    item["luma_mean"].add(float(np.mean(luma)))
                    item["luma_max"].add(float(np.max(luma)))
                    item["bright_ratio_ge_220"].add(float(np.mean(luma >= 220.0)))
                    item["nonzero_ratio"].add(float(np.mean(luma > 0.0)))
                    prev = previous_luma.get(topic)
                    if prev is not None and prev.size == luma.size:
                        diff = float(np.mean(np.abs(luma.astype(np.float32) - prev.astype(np.float32))))
                        add_top(
                            item["top_frame_diffs"],
                            {"t_s": round(rel_t, 3), "mean_abs_luma_diff": round(diff, 4)},
                            key="mean_abs_luma_diff",
                        )
                    previous_luma[topic] = luma.copy()
            elif topic in PLAN_TOPICS:
                poses = list(msg.poses)
                points = [
                    (float(pose.pose.position.x), float(pose.pose.position.y))
                    for pose in poses
                ]
                plan = plans[topic]
                plan["messages"] += 1
                plan["poses"].add(len(poses))
                if len(points) >= 2:
                    plan["length_m"].add(path_length(points))
            elif topic in TF_TOPICS:
                tf_item = tf_summary[topic]
                tf_item["messages"] += 1
                if tf_item["first_bag_s"] is None:
                    tf_item["first_bag_s"] = round(rel_t, 3)
                tf_item["last_bag_s"] = round(rel_t, 3)
                for transform in msg.transforms:
                    parent = str(transform.header.frame_id)
                    child = str(transform.child_frame_id)
                    pair = f"{parent}->{child}"
                    tf_item["transform_count"] += 1
                    tf_item["pairs"][pair] += 1
                    tf_item["parent_frames"][parent] += 1
                    tf_item["child_frames"][child] += 1
                    stamp_ns = (
                        int(transform.header.stamp.sec) * 1_000_000_000
                        + int(transform.header.stamp.nanosec)
                    )
                    if start_ns is not None and stamp_ns > 0:
                        stamp_s = round((stamp_ns - start_ns) / 1e9, 3)
                        if tf_item["first_stamp_s"] is None:
                            tf_item["first_stamp_s"] = stamp_s
                        tf_item["last_stamp_s"] = stamp_s

    if encoder_first is not None and encoder_last is not None:
        encoder_summary.update({
            "left_count_delta": int(encoder_last.left_motor_count) - int(encoder_first.left_motor_count),
            "right_count_delta": int(encoder_last.right_motor_count) - int(encoder_first.right_motor_count),
            "last_left_rpm": int(encoder_last.left_motor_rpm),
            "last_right_rpm": int(encoder_last.right_motor_rpm),
        })

    result["cmd_vel"] = summarize_cmd(cmd_samples)
    result["odometry"] = summarize_odom(odom_samples)
    result["encoders"] = encoder_summary
    result["line_detection"] = {
        "diagnostic_reason_counts": dict(line_diag_reasons.most_common()),
        "diagnostic_numeric_stats": stats_dict(line_diag_stats),
        "diagnostic_samples": line_diag_samples,
        "line_pixels": {
            "messages": line_pixels["messages"],
            "pixel_count": line_pixels["pixel_count"].as_dict(),
            "first_nonzero_s": line_pixels["first_nonzero_s"],
            "max_pixel_count": line_pixels["max_pixel_count"],
        },
        "line_points": {
            "messages": line_points["messages"],
            "point_count": line_points["point_count"].as_dict(),
            "first_nonzero_s": line_points["first_nonzero_s"],
            "max_point_count": line_points["max_point_count"],
            "frames": dict(line_points["frames"].most_common()),
        },
    }
    result["lidar_line_detection"] = {
        "diagnostic_reason_counts": dict(lidar_diag_reasons.most_common()),
        "line_points": {
            "messages": lidar_line_points["messages"],
            "point_count": lidar_line_points["point_count"].as_dict(),
            "first_nonzero_s": lidar_line_points["first_nonzero_s"],
            "max_point_count": lidar_line_points["max_point_count"],
            "frames": dict(lidar_line_points["frames"].most_common()),
        },
    }
    result["costmaps"] = {
        topic: {
            "messages": value["messages"],
            "nonzero_cells": value["nonzero_cells"].as_dict(),
            "hard_cells": value["hard_cells"].as_dict(),
            "first_nonzero_s": value["first_nonzero_s"],
            "max_value": value["max_value"],
        }
        for topic, value in sorted(costmaps.items())
        if value["messages"] or topic in topics_meta
    }
    result["scans"] = {
        topic: {
            "messages": value["messages"],
            "finite_ranges": value["finite_ranges"].as_dict(),
            "close_ranges_lt_1m": value["close_ranges_lt_1m"].as_dict(),
            "min_range_m": value["min_range_m"].as_dict(),
            "closest_events": value["closest_events"],
        }
        for topic, value in sorted(scans.items())
        if value["messages"] or topic in topics_meta
    }
    result["pointclouds"] = {
        topic: {
            "messages": value["messages"],
            "points": value["points"].as_dict(),
            "first_nonzero_s": value["first_nonzero_s"],
        }
        for topic, value in sorted(pointclouds.items())
        if value["messages"] or topic in topics_meta
    }
    result["images"] = {
        topic: {
            "messages": value["messages"],
            "sampled": value["sampled"],
            "encoding": dict(value["encoding"].most_common()),
            "frame_ids": dict(value["frame_ids"].most_common()),
            "width": value["width"],
            "height": value["height"],
            "luma_mean": value["luma_mean"].as_dict(),
            "luma_max": value["luma_max"].as_dict(),
            "bright_ratio_ge_220": value["bright_ratio_ge_220"].as_dict(),
            "nonzero_ratio": value["nonzero_ratio"].as_dict(),
            "top_frame_diffs": value["top_frame_diffs"],
        }
        for topic, value in sorted(images.items())
        if value["messages"] or topic in topics_meta
    }
    result["plans"] = {
        topic: {
            "messages": value["messages"],
            "poses": value["poses"].as_dict(),
            "length_m": value["length_m"].as_dict(),
        }
        for topic, value in sorted(plans.items())
        if value["messages"] or topic in topics_meta
    }
    result["tf"] = {}
    for topic, value in sorted(tf_summary.items()):
        if not value["messages"] and topic not in topics_meta:
            continue
        pairs = dict(value["pairs"].most_common(30))
        camera_pairs = {
            pair: count for pair, count in pairs.items()
            if any(token in pair.lower() for token in ("zed", "camera", "rgb", "depth"))
        }
        result["tf"][topic] = {
            "messages": value["messages"],
            "transform_count": value["transform_count"],
            "first_bag_s": value["first_bag_s"],
            "last_bag_s": value["last_bag_s"],
            "first_stamp_s": value["first_stamp_s"],
            "last_stamp_s": value["last_stamp_s"],
            "top_pairs": pairs,
            "camera_pairs": camera_pairs,
            "top_child_frames": dict(value["child_frames"].most_common(30)),
            "top_parent_frames": dict(value["parent_frames"].most_common(30)),
        }
    result["decode_errors"] = dict(result["decode_errors"].most_common())
    return result


def derive_findings(runs: list[dict[str, Any]]) -> list[str]:
    findings: list[str] = []
    camera_runs = [
        run for run in runs
        if run["name"].startswith("camera_line_detection") or run["name"].startswith("manual_course")
    ]
    if camera_runs:
        runs_with_pixels = [
            run for run in camera_runs
            if int(run["line_detection"]["line_pixels"].get("max_pixel_count") or 0) > 0
        ]
        runs_with_points = [
            run for run in camera_runs
            if int(run["line_detection"]["line_points"].get("max_point_count") or 0) > 0
        ]
        zero_points_with_pixels = [
            run for run in runs_with_pixels
            if int(run["line_detection"]["line_points"].get("max_point_count") or 0) == 0
        ]
        max_pixels = max(
            (int(run["line_detection"]["line_pixels"].get("max_pixel_count") or 0)
             for run in camera_runs),
            default=0,
        )
        max_points = max(
            (int(run["line_detection"]["line_points"].get("max_point_count") or 0)
             for run in camera_runs),
            default=0,
        )
        if max_pixels > 0 and max_points == 0:
            findings.append(
                "Camera detector produced 2-D line pixels in real bags but published zero /line_points; line avoidance cannot work until projection/TF/depth synchronization is fixed."
            )
        elif max_points > 0:
            findings.append(
                "Camera detector published non-empty /line_points in %d/%d camera/perception bags; the working run was: %s."
                % (
                    len(runs_with_points),
                    len(camera_runs),
                    ", ".join(run["name"] for run in runs_with_points),
                )
            )
            if zero_points_with_pixels:
                findings.append(
                    "%d/%d camera/perception bags had non-empty line_pixels but zero /line_points: %s. In those runs /line_costmap stayed empty, so Nav2 could drive over visible tape."
                    % (
                        len(zero_points_with_pixels),
                        len(camera_runs),
                        ", ".join(run["name"] for run in zero_points_with_pixels),
                    )
                )
        else:
            findings.append(
                "Camera detector did not produce line pixels or line points in the selected real bags."
            )
        reasons = Counter()
        for run in camera_runs:
            reasons.update(run["line_detection"]["diagnostic_reason_counts"])
        if reasons:
            top_reason, top_count = reasons.most_common(1)[0]
            findings.append(
                "Dominant camera line diagnostic reason: %s (%d messages)."
                % (top_reason, top_count)
            )
        runs_with_raw_zed = [
            run for run in camera_runs
            if int(
                run.get("images", {})
                .get("/zed/zed_node/rgb/color/rect/image", {})
                .get("messages")
                or 0
            ) > 0
            and int(
                run.get("images", {})
                .get("/zed/zed_node/depth/depth_registered", {})
                .get("messages")
                or 0
            ) > 0
        ]
        if len(runs_with_raw_zed) < len(camera_runs):
            missing_raw = [
                run["name"] for run in camera_runs
                if run not in runs_with_raw_zed
            ]
            findings.append(
                "Only %d/%d camera/perception bags include raw ZED RGB+depth image topics; missing raw ZED image data in: %s."
                % (
                    len(runs_with_raw_zed),
                    len(camera_runs),
                    ", ".join(missing_raw),
                )
            )
    plan_message_count = 0
    for run in runs:
        for plan in run.get("plans", {}).values():
            plan_message_count += int(plan.get("messages") or 0)
    if plan_message_count == 0:
        findings.append(
            "Selected manual/calibration bags contain zero Nav2 plan messages, so they cannot validate planner-vs-costmap clearance or path-through-line behavior; use an autonomous nav debug bag or a live sim run for that check."
        )
    yaw = next((run for run in runs if run["name"] == "in_place_yaw_ladder_1"), None)
    if yaw:
        cmd = yaw.get("cmd_vel", {}).get("/cmd_vel", {})
        odom_values = yaw.get("odometry", {}).values()
        max_w = float(cmd.get("max_abs_angular_radps") or 0.0)
        max_yaw = max(
            (abs(float(odom.get("yaw_delta_deg") or 0.0)) for odom in odom_values),
            default=0.0,
        )
        if max_w > 0.1 and max_yaw < 1.0:
            findings.append(
                "in_place_yaw_ladder_1 commanded yaw up to %.2f rad/s, but odom yaw changed only %.3f deg; do not tune Gazebo yaw dynamics from this bag until the odom/encoder yaw path is explained."
                % (max_w, max_yaw)
            )
    straight = next((run for run in runs if run["name"] == "straight_distance_drift_low_2"), None)
    if straight:
        phys = straight.get("physical_measurement") or {}
        odom = straight.get("odometry", {}).get("/odom") or {}
        if phys.get("forward_m") and odom.get("start_frame_forward_m"):
            ratio = float(phys["forward_m"]) / float(odom["start_frame_forward_m"])
            findings.append(
                "straight_distance_drift_low_2 physical/odom forward ratio = %.3f; treat as odom-scale diagnostic, not lateral-bias tuning."
                % ratio
            )
    return findings


def render_markdown(report: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append("# Gazebo Canon Offline Report")
    lines.append("")
    lines.append("Generated by `scripts/analyze_gazebo_canon_offline.py`.")
    lines.append("")
    lines.append("## Executive Findings")
    lines.append("")
    for finding in report["findings"]:
        lines.append(f"- {finding}")
    if not report["findings"]:
        lines.append("- No derived findings.")
    lines.append("")
    lines.append("## Run Inventory")
    lines.append("")
    lines.append("| Run | Category | Bag s | Video s | Msgs | Topics | Missing/zero expected |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: |")
    for run in report["runs"]:
        video_s = run["video"].get("duration_s", "NA")
        zero = len(run["topic_health"].get("zero_count_expected_topics", []))
        missing = len(run["topic_health"].get("missing_at_start", []))
        lines.append(
            "| {name} | {cat} | {bag_s} | {video_s} | {msgs} | {topics} | {missing}/{zero} |".format(
                name=run["name"],
                cat=run["category"],
                bag_s=run["bag_metadata"].get("duration_s", "NA"),
                video_s=video_s,
                msgs=run["bag_metadata"].get("message_count", "NA"),
                topics=run["bag_metadata"].get("topic_count", "NA"),
                missing=missing,
                zero=zero,
            )
        )
    lines.append("")
    lines.append("## Dynamics Summary")
    lines.append("")
    for run in report["runs"]:
        if run["category"] != "dynamics":
            continue
        lines.append(f"### {run['name']}")
        if run.get("physical_measurement"):
            lines.append("")
            lines.append(f"- Physical measurement: `{run['physical_measurement']}`")
        for topic, odom in run.get("odometry", {}).items():
            lines.append(
                "- {topic}: count={count}, path={path} m, forward={forward} m, lateral={lateral} m, yaw={yaw} deg".format(
                    topic=topic,
                    count=odom.get("count"),
                    path=odom.get("path_length_m"),
                    forward=odom.get("start_frame_forward_m"),
                    lateral=odom.get("start_frame_lateral_m"),
                    yaw=odom.get("yaw_delta_deg"),
                )
            )
        for topic, cmd in run.get("cmd_vel", {}).items():
            lines.append(
                "- {topic}: count={count}, nonzero={nonzero}, max_v={max_v}, max_w={max_w}".format(
                    topic=topic,
                    count=cmd.get("count"),
                    nonzero=cmd.get("nonzero_count"),
                    max_v=cmd.get("max_abs_linear_mps"),
                    max_w=cmd.get("max_abs_angular_radps"),
                )
            )
        lines.append("")
    lines.append("## Camera Line Detection Summary")
    lines.append("")
    for run in report["runs"]:
        if run["category"] != "full_perception":
            continue
        ld = run["line_detection"]
        lpix = ld["line_pixels"]
        lpts = ld["line_points"]
        reasons = ld["diagnostic_reason_counts"]
        top_reason = next(iter(reasons.items()), ("none", 0))
        lines.append(
            "- {name}: max line_pixels={pix}, max line_points={pts}, first pixels={fp}, first points={fpt}, top reason={reason} ({count})".format(
                name=run["name"],
                pix=lpix.get("max_pixel_count"),
                pts=lpts.get("max_point_count"),
                fp=lpix.get("first_nonzero_s"),
                fpt=lpts.get("first_nonzero_s"),
                reason=top_reason[0],
                count=top_reason[1],
            )
        )
        rgb = run.get("images", {}).get("/zed/zed_node/rgb/color/rect/image", {})
        depth = run.get("images", {}).get("/zed/zed_node/depth/depth_registered", {})
        if rgb or depth:
            lines.append(
                "  - image frames: rgb={rgb}, depth={depth}".format(
                    rgb=list((rgb.get("frame_ids") or {}).items())[:3],
                    depth=list((depth.get("frame_ids") or {}).items())[:3],
                )
            )
    lines.append("")
    lines.append("## Costmap And Planning Summary")
    lines.append("")
    any_plan_messages = any(
        int(plan.get("messages") or 0) > 0
        for run in report["runs"]
        for plan in run.get("plans", {}).values()
    )
    if not any_plan_messages:
        lines.append(
            "The selected bags contain no Nav2 plan messages. Costmap occupancy is summarized below, but plan/costmap collision clearance cannot be evaluated from this dataset."
        )
        lines.append("")
    for run in report["runs"]:
        interesting = []
        for topic in ("/line_costmap", "/lidar_line_costmap", "/local_costmap/costmap", "/global_costmap/costmap"):
            cm = run.get("costmaps", {}).get(topic)
            if not cm:
                continue
            interesting.append(
                "{topic}: msgs={msgs}, max_nonzero={max_nonzero}, first_nonzero={first}".format(
                    topic=topic,
                    msgs=cm.get("messages"),
                    max_nonzero=cm.get("nonzero_cells", {}).get("max"),
                    first=cm.get("first_nonzero_s"),
                )
            )
        if interesting:
            lines.append(f"### {run['name']}")
            for item in interesting:
                lines.append(f"- {item}")
            for topic, plan in run.get("plans", {}).items():
                if plan.get("messages"):
                    lines.append(
                        "- {topic}: msgs={msgs}, poses={poses}, length_m={length}".format(
                            topic=topic,
                            msgs=plan.get("messages"),
                            poses=plan.get("poses", {}).get("mean"),
                            length=plan.get("length_m", {}).get("mean"),
                        )
                    )
            lines.append("")
    lines.append("## TF Summary")
    lines.append("")
    for run in report["runs"]:
        tf = run.get("tf", {})
        if not tf:
            continue
        lines.append(f"### {run['name']}")
        for topic, summary in tf.items():
            camera_pairs = summary.get("camera_pairs") or {}
            pairs_preview = camera_pairs or summary.get("top_pairs", {})
            preview_items = list(pairs_preview.items())[:6]
            lines.append(
                "- {topic}: msgs={msgs}, transforms={transforms}, bag_window={first}->{last}, stamp_window={sfirst}->{slast}, pairs={pairs}".format(
                    topic=topic,
                    msgs=summary.get("messages"),
                    transforms=summary.get("transform_count"),
                    first=summary.get("first_bag_s"),
                    last=summary.get("last_bag_s"),
                    sfirst=summary.get("first_stamp_s"),
                    slast=summary.get("last_stamp_s"),
                    pairs=preview_items,
                )
            )
        lines.append("")
    lines.append("## Sync Candidates")
    lines.append("")
    lines.append("Use these bag-relative timestamps to align the phone videos with the hand-wave.")
    lines.append("")
    for run in report["runs"]:
        candidates = []
        for topic, image in run.get("images", {}).items():
            if image.get("top_frame_diffs"):
                candidates.append((topic, image["top_frame_diffs"][:3]))
        scan = run.get("scans", {}).get("/scan_fullframe", {})
        if scan.get("closest_events"):
            candidates.append(("/scan_fullframe closest", scan["closest_events"][:3]))
        if candidates:
            lines.append(f"### {run['name']}")
            for topic, items in candidates:
                lines.append(f"- {topic}: `{items}`")
            lines.append("")
    lines.append("## Full JSON")
    lines.append("")
    lines.append("See `docs/gazebo_canon_offline_report.json` for the full machine-readable summary.")
    lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default="/Users/cole/code/git/AutoNavB")
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--max-image-samples", type=int, default=200)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    repo = Path(args.repo).expanduser().resolve()
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    typestore = register_custom_types(repo)
    runs = [
        analyze_run(run, repo, typestore, max_image_samples=args.max_image_samples)
        for run in RUNS
    ]
    report = {
        "repo": str(repo),
        "runs": runs,
        "findings": derive_findings(runs),
    }
    if args.output_json:
        output_json = Path(args.output_json).expanduser()
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown = render_markdown(report)
    if args.output_md:
        output_md = Path(args.output_md).expanduser()
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(markdown, encoding="utf-8")
    else:
        print(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
