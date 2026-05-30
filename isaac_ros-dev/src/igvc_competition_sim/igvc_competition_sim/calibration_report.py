#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys
from typing import Any

from .dynamics_config import (
    default_calibration_path,
    load_config,
    segment_angular_radps,
    segment_linear_mps,
)
from .course import load_yaml


def _load_mapping(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8").strip()
    if text.startswith("{"):
        data = json.loads(text)
        if not isinstance(data, dict):
            raise ValueError("%s did not load as a mapping" % path)
        return data
    return load_yaml(path)


def _read_metadata(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False}
    try:
        data = load_yaml(path)
    except Exception as exc:
        parsed = _read_metadata_text_fallback(path)
        parsed["yaml_error"] = str(exc)
        return parsed
    info = data.get("rosbag2_bagfile_information", {})
    topics: dict[str, int] = {}
    for item in info.get("topics_with_message_count", []):
        meta = item.get("topic_metadata", {})
        name = meta.get("name")
        if name:
            topics[str(name)] = int(item.get("message_count", 0))
    return {
        "present": True,
        "duration_s": float(info.get("duration", {}).get("nanoseconds", 0)) / 1e9,
        "message_count": int(info.get("message_count", 0)),
        "topics": topics,
    }


def _read_metadata_text_fallback(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    duration_match = re.search(
        r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    message_match = re.search(r"^\s*message_count:\s*(\d+)", text, re.MULTILINE)
    topics: dict[str, int] = {}
    current_name: str | None = None
    for raw in text.splitlines():
        stripped = raw.strip()
        if stripped.startswith("name: /"):
            current_name = stripped.split(":", 1)[1].strip()
        elif current_name and stripped.startswith("message_count:"):
            topics[current_name] = int(stripped.split(":", 1)[1].strip())
            current_name = None
    return {
        "present": True,
        "duration_s": (
            float(duration_match.group(1)) / 1e9 if duration_match else None
        ),
        "message_count": int(message_match.group(1)) if message_match else None,
        "topics": topics,
    }


def _read_run_metadata(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.startswith("  "):
            continue
        key, value = line.split(":", 1)
        out[key.strip()] = value.strip()
    return out


def _read_command_metrics(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def _float_row(row: dict[str, Any], key: str) -> float | None:
    value = row.get(key)
    if value in (None, ""):
        return None
    return float(value)


def _summarize_command_metrics(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in rows:
        out.append({
            "segment": row.get("segment", ""),
            "target_distance_m": _float_row(row, "target_distance_m"),
            "elapsed_s": _float_row(row, "elapsed_s"),
            "command_linear_mps": _float_row(row, "command_linear_mps"),
            "command_angular_radps": _float_row(row, "command_angular_radps"),
            "odom_forward_m": _float_row(row, "forward_m"),
            "odom_lateral_m": _float_row(row, "lateral_m"),
            "heading_drift_deg": _float_row(row, "heading_drift_deg"),
            "overshoot_m": _float_row(row, "overshoot_m"),
        })
    return out


def _video_info(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False}
    stat = path.stat()
    return {
        "present": True,
        "path": str(path),
        "size_bytes": stat.st_size,
        "mtime_epoch_s": stat.st_mtime,
    }


def _yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _nearest_pose(samples: list[dict[str, float]], stamp_s: float) -> dict[str, float] | None:
    if not samples:
        return None
    return min(samples, key=lambda sample: abs(sample["t"] - stamp_s))


def _cluster_commands(samples: list[dict[str, float]]) -> list[dict[str, float]]:
    if not samples:
        return []
    clusters: list[dict[str, float]] = []
    current = {
        "start_s": samples[0]["t"],
        "end_s": samples[0]["t"],
        "linear_mps": samples[0]["v"],
        "angular_radps": samples[0]["w"],
        "samples": 1,
    }
    for sample in samples[1:]:
        if (
            abs(sample["v"] - current["linear_mps"]) < 1e-4
            and abs(sample["w"] - current["angular_radps"]) < 1e-4
        ):
            current["end_s"] = sample["t"]
            current["samples"] += 1
            continue
        clusters.append(current)
        current = {
            "start_s": sample["t"],
            "end_s": sample["t"],
            "linear_mps": sample["v"],
            "angular_radps": sample["w"],
            "samples": 1,
        }
    clusters.append(current)
    return [
        {**cluster, "duration_s": cluster["end_s"] - cluster["start_s"]}
        for cluster in clusters
        if cluster["samples"] >= 3
    ]


def _try_rosbag_motion_summary(bag_dir: Path,
                               expected_segments: list[dict[str, Any]]
                               ) -> dict[str, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        return _try_rosbags_motion_summary(bag_dir, expected_segments, str(exc))

    bag_path = bag_dir / "bag"
    if not bag_path.is_dir():
        return {"available": False, "reason": "bag directory missing"}

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    needed = {"/cmd_vel", "/odom", "/local_ekf/odom"}
    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in needed
        if topic in topic_types
    }
    cmd_samples: list[dict[str, float]] = []
    odom_samples: dict[str, list[dict[str, float]]] = {
        "/odom": [],
        "/local_ekf/odom": [],
    }
    first_ts: int | None = None
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in msg_types:
            continue
        if first_ts is None:
            first_ts = ts
        t = (ts - first_ts) / 1e9
        msg = deserialize_message(data, msg_types[topic])
        if topic == "/cmd_vel":
            cmd_samples.append({
                "t": t,
                "v": float(msg.linear.x),
                "w": float(msg.angular.z),
            })
        elif topic in odom_samples:
            odom_samples[topic].append({
                "t": t,
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "yaw": _yaw_from_quaternion(msg.pose.pose.orientation),
                "v": float(msg.twist.twist.linear.x),
                "w": float(msg.twist.twist.angular.z),
            })

    return _motion_summary_from_samples(
        cmd_samples, odom_samples, expected_segments, backend="rosbag2_py")


def _try_rosbags_motion_summary(bag_dir: Path,
                                expected_segments: list[dict[str, Any]],
                                rosbag2_error: str) -> dict[str, Any]:
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
    except Exception as exc:
        return {
            "available": False,
            "reason": (
                "ROS bag decode unavailable: rosbag2_py=%s; rosbags=%s"
                % (rosbag2_error, exc)
            ),
        }

    bag_path = bag_dir / "bag"
    if not bag_path.is_dir():
        return {"available": False, "reason": "bag directory missing"}

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    cmd_samples: list[dict[str, float]] = []
    odom_samples: dict[str, list[dict[str, float]]] = {
        "/odom": [],
        "/local_ekf/odom": [],
    }
    first_ts: int | None = None
    with Reader(bag_path) as reader:
        connections = [
            connection for connection in reader.connections
            if connection.topic in ("/cmd_vel", "/odom", "/local_ekf/odom")
        ]
        for connection, timestamp, raw in reader.messages(connections=connections):
            if first_ts is None:
                first_ts = timestamp
            rel_s = (timestamp - first_ts) / 1e9
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            if connection.topic == "/cmd_vel":
                cmd_samples.append({
                    "t": rel_s,
                    "v": float(msg.linear.x),
                    "w": float(msg.angular.z),
                })
            elif connection.topic in odom_samples:
                odom_samples[connection.topic].append({
                    "t": rel_s,
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": _yaw_from_quaternion(msg.pose.pose.orientation),
                    "v": float(msg.twist.twist.linear.x),
                    "w": float(msg.twist.twist.angular.z),
                })

    return _motion_summary_from_samples(
        cmd_samples, odom_samples, expected_segments, backend="rosbags")


def _motion_summary_from_samples(
    cmd_samples: list[dict[str, float]],
    odom_samples: dict[str, list[dict[str, float]]],
    expected_segments: list[dict[str, Any]],
    backend: str,
) -> dict[str, Any]:
    nonzero_intervals = [
        interval for interval in _cluster_commands(cmd_samples)
        if abs(interval["linear_mps"]) > 1e-3
        or abs(interval["angular_radps"]) > 1e-3
    ]
    nonzero_expected = [
        segment for segment in expected_segments
        if abs(segment_linear_mps(segment)) > 1e-3
        or abs(segment_angular_radps(segment)) > 1e-3
    ]

    paired: list[dict[str, Any]] = []
    for idx, interval in enumerate(nonzero_intervals[:len(nonzero_expected)]):
        expected = nonzero_expected[idx]
        item: dict[str, Any] = {
            "label": expected.get("label", "segment_%d" % idx),
            "command_interval": interval,
            "expected_linear_mps": segment_linear_mps(expected),
            "expected_angular_radps": segment_angular_radps(expected),
            "odom": {},
        }
        for topic, samples in odom_samples.items():
            start = _nearest_pose(samples, interval["start_s"])
            end = _nearest_pose(samples, interval["end_s"])
            if start is None or end is None:
                continue
            dx = end["x"] - start["x"]
            dy = end["y"] - start["y"]
            c = math.cos(start["yaw"])
            s = math.sin(start["yaw"])
            item["odom"][topic] = {
                "forward_m": c * dx + s * dy,
                "lateral_m": -s * dx + c * dy,
                "euclidean_m": math.hypot(dx, dy),
                "heading_drift_rad": _angle_diff(end["yaw"], start["yaw"]),
                "heading_drift_deg": math.degrees(_angle_diff(end["yaw"], start["yaw"])),
            }
        paired.append(item)

    return {
        "available": True,
        "backend": backend,
        "cmd_interval_count": len(nonzero_intervals),
        "paired_nonzero_segments": paired,
    }


def build_report(config_path: Path,
                 bag_root_override: str | None,
                 video_root_override: str | None,
                 decode_rosbag: bool) -> dict[str, Any]:
    config = load_config(config_path)
    source = config.get("source_runs", {})
    bag_root = Path(bag_root_override or source.get("bag_root", "")).expanduser()
    video_root = Path(video_root_override or source.get("video_root", "")).expanduser()

    runs: dict[str, Any] = {}
    for run_name, run_cfg in source.get("runs", {}).items():
        run_dir = bag_root / str(run_cfg.get("bag_dir", run_name))
        video = video_root / str(run_cfg.get("video_file", run_name + ".MOV"))
        run_metadata = _read_run_metadata(run_dir / "run_metadata.txt")
        metrics = _read_command_metrics(run_dir / "command_metrics.csv")
        profile_segments: list[dict[str, Any]] = []
        profiles_path = run_dir / "profiles.yaml"
        profile_name = str(run_cfg.get("profile", run_metadata.get("profile", "")))
        if profiles_path.is_file() and profile_name:
            profiles = _load_mapping(profiles_path)
            profile_segments = list(
                profiles.get("profiles", {}).get(profile_name, {}).get("segments", []))

        run_report: dict[str, Any] = {
            "bag_dir": str(run_dir),
            "video": _video_info(video),
            "run_metadata": run_metadata,
            "bag_metadata": _read_metadata(run_dir / "bag" / "metadata.yaml"),
            "command_metrics": _summarize_command_metrics(metrics),
            "use_for_default_tuning": run_cfg.get("use_for_default_tuning", []),
            "data_quality_notes": run_cfg.get("data_quality_notes", []),
            "excluded_from_default_tuning": run_cfg.get(
                "excluded_from_default_tuning", []),
            "exclusion_reason": run_cfg.get("exclusion_reason", ""),
            "physical_measurement": run_cfg.get("physical_measurement", {}),
        }
        if decode_rosbag:
            run_report["rosbag_motion_summary"] = _try_rosbag_motion_summary(
                run_dir, profile_segments)
        runs[run_name] = run_report

    return {
        "calibration_config": str(config_path),
        "bag_root": str(bag_root),
        "video_root": str(video_root),
        "straight_lateral_drift_policy": (
            "excluded_from_default_flat_ground_tuning_due_to_cross_slope"
        ),
        "competition_constraints": config.get("competition_constraints", {}),
        "dynamics_model": config.get("dynamics_model", {}),
        "odometry_model": config.get("odometry_model", {}),
        "runs": runs,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibration-config",
        default=str(default_calibration_path()),
    )
    parser.add_argument("--bag-root", default=None)
    parser.add_argument("--video-root", default=None)
    parser.add_argument("--output", default="")
    parser.add_argument(
        "--decode-rosbag",
        action="store_true",
        help="Decode /cmd_vel and odom topics with rosbag2_py or optional rosbags.",
    )
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    report = build_report(
        Path(args.calibration_config).expanduser().resolve(),
        args.bag_root,
        args.video_root,
        args.decode_rosbag,
    )
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
