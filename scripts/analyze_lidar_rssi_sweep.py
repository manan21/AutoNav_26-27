#!/usr/bin/env python3
"""Sweep SICK RSSI thresholds for plain-white-tape line detection.

Run inside the ROS 2 environment that has this workspace sourced:

    python3 scripts/analyze_lidar_rssi_sweep.py /path/to/bag

The analyzer reads raw /cloud_all_fields_fullframe samples once, applies the
same ground gate, adaptive RSSI candidate selection, and tape-shape clustering
used by lidar_line_detector, then ranks threshold combinations. It is designed
for a single short course-access bag.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Iterable

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


@dataclass
class Sample:
    x: float
    y: float
    z: float
    intensity: float
    range: float
    layer: int
    echo: int
    reflector: bool


@dataclass
class RunningStats:
    count: int = 0
    total: float = 0.0
    total_sq: float = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        self.total += value
        self.total_sq += value * value

    @property
    def mean(self) -> float:
        return self.total / self.count if self.count else 0.0

    @property
    def stddev(self) -> float:
        if self.count < 2:
            return 0.0
        variance = max(0.0, self.total_sq / self.count - self.mean * self.mean)
        return math.sqrt(variance)


@dataclass
class ClusterGeometry:
    cx: float
    cy: float
    major_x: float
    major_y: float
    minor_x: float
    minor_y: float
    major_min: float
    major_max: float
    minor_min: float
    minor_max: float
    mean_z: float

    @property
    def length(self) -> float:
        return self.major_max - self.major_min

    @property
    def width(self) -> float:
        return self.minor_max - self.minor_min

    @property
    def aspect(self) -> float:
        return self.length / max(self.width, 0.01)


@dataclass(frozen=True)
class SweepParams:
    min_intensity: float
    adaptive_range_bin_m: float
    adaptive_min_delta: float
    adaptive_stddev_multiplier: float


@dataclass
class SweepResult:
    params: SweepParams
    clouds: int = 0
    ground_samples: int = 0
    candidates: int = 0
    clusters: int = 0
    accepted_clusters: int = 0
    rejected_clusters: int = 0
    accepted_points: int = 0
    accepted_length_m: float = 0.0
    positive_length_m: float = 0.0
    negative_length_m: float = 0.0
    longest_cluster_m: float = 0.0
    median_width_m: float = 0.0
    cluster_rows: list[dict] | None = None

    def score(self) -> tuple:
        # Prefer consistent accepted tape-like length while penalizing floods.
        flood_penalty = self.candidates / max(1, self.ground_samples)
        if self.positive_length_m > 0.0 or self.negative_length_m > 0.0:
            return (
                self.positive_length_m,
                -self.negative_length_m,
                self.accepted_length_m,
                self.accepted_clusters,
                -flood_penalty,
                -self.rejected_clusters,
            )
        return (
            self.accepted_length_m,
            self.accepted_clusters,
            -flood_penalty,
            -self.rejected_clusters,
        )


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def get_topic_types(bag_path: str) -> dict[str, str]:
    reader = open_reader(bag_path)
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def quaternion_matrix(q) -> list[list[float]]:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n <= 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return [
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ]


def transform_matrix(tf_msg) -> list[list[float]]:
    r = quaternion_matrix(tf_msg.transform.rotation)
    t = tf_msg.transform.translation
    return [
        [r[0][0], r[0][1], r[0][2], t.x],
        [r[1][0], r[1][1], r[1][2], t.y],
        [r[2][0], r[2][1], r[2][2], t.z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def invert_matrix(m: list[list[float]]) -> list[list[float]]:
    r_t = [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
    t = [m[0][3], m[1][3], m[2][3]]
    inv_t = [
        -(r_t[0][0] * t[0] + r_t[0][1] * t[1] + r_t[0][2] * t[2]),
        -(r_t[1][0] * t[0] + r_t[1][1] * t[1] + r_t[1][2] * t[2]),
        -(r_t[2][0] * t[0] + r_t[2][1] * t[1] + r_t[2][2] * t[2]),
    ]
    return [
        [r_t[0][0], r_t[0][1], r_t[0][2], inv_t[0]],
        [r_t[1][0], r_t[1][1], r_t[1][2], inv_t[1]],
        [r_t[2][0], r_t[2][1], r_t[2][2], inv_t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def transform_point(m: list[list[float]], x: float, y: float, z: float) -> tuple[float, float, float]:
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


class TransformGraph:
    def __init__(self) -> None:
        self.edges: dict[tuple[str, str], list[list[float]]] = {}

    def add(self, tf_msg) -> None:
        parent = tf_msg.header.frame_id.strip("/")
        child = tf_msg.child_frame_id.strip("/")
        if parent and child:
            self.edges[(parent, child)] = transform_matrix(tf_msg)

    def lookup(self, target: str, source: str) -> list[list[float]] | None:
        target = target.strip("/")
        source = source.strip("/")
        if target == source:
            return [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]

        queue: deque[tuple[str, list[list[float]]]] = deque()
        queue.append((source, self.lookup(source, source)))
        seen = {source}
        while queue:
            frame, mat_source_to_frame = queue.popleft()
            if frame == target:
                return mat_source_to_frame
            for (parent, child), mat_child_to_parent in self.edges.items():
                if child == frame and parent not in seen:
                    seen.add(parent)
                    queue.append((parent, matmul(mat_child_to_parent, mat_source_to_frame)))
                if parent == frame and child not in seen:
                    seen.add(child)
                    queue.append((child, matmul(invert_matrix(mat_child_to_parent), mat_source_to_frame)))
        return None


def build_tf_graph(bag_path: str, topic_types: dict[str, str]) -> TransformGraph:
    graph = TransformGraph()
    tf_topics = [topic for topic in ("/tf_static", "/tf") if topic in topic_types]
    if not tf_topics:
        return graph
    msg_types = {topic: get_message(topic_types[topic]) for topic in tf_topics}
    reader = open_reader(bag_path)
    while reader.has_next():
        topic, data, _stamp_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        for tf_msg in msg.transforms:
            graph.add(tf_msg)
    return graph


def field_names(msg) -> set[str]:
    return {field.name for field in msg.fields}


def choose_field(names: set[str], preferred: str, fallback: str | None = None) -> str | None:
    if preferred in names:
        return preferred
    if fallback and fallback in names:
        return fallback
    return None


def stats_key(sample: Sample, adaptive_range_bin_m: float, normalize_by_layer: bool) -> tuple[int, int]:
    range_bin = int(math.floor(sample.range / adaptive_range_bin_m))
    layer_bin = sample.layer if normalize_by_layer and sample.layer >= 0 else 0
    return layer_bin, range_bin


def pass_layer_echo(sample: Sample, args: argparse.Namespace) -> bool:
    if args.layer_min >= 0 and sample.layer >= 0 and sample.layer < args.layer_min:
        return False
    if args.layer_max >= 0 and sample.layer >= 0 and sample.layer > args.layer_max:
        return False
    if args.echo_filter >= 0 and sample.echo >= 0 and sample.echo != args.echo_filter:
        return False
    return True


def extract_samples(msg, transform, args: argparse.Namespace) -> list[Sample]:
    names = field_names(msg)
    intensity_name = choose_field(names, args.intensity_field, "intensity")
    if not {"x", "y", "z"}.issubset(names) or intensity_name is None:
        raise RuntimeError(
            f"{args.topic} missing x/y/z/{args.intensity_field}; fields={sorted(names)}"
        )

    selected = ["x", "y", "z", intensity_name]
    optional = []
    for name in ("range", "layer", "ring", "echo", "reflector"):
        if name in names:
            optional.append(name)
    selected.extend(optional)

    samples: list[Sample] = []
    for row_index, point in enumerate(
        point_cloud2.read_points(msg, field_names=selected, skip_nans=True)
    ):
        if args.point_stride > 1 and row_index % args.point_stride:
            continue
        if hasattr(point, "dtype") and getattr(point.dtype, "names", None):
            values = {name: point[name].item() for name in selected}
        else:
            values = dict(zip(selected, point))
        lx = float(values["x"])
        ly = float(values["y"])
        lz = float(values["z"])
        intensity = float(values[intensity_name])
        bx, by, bz = transform_point(transform, lx, ly, lz)
        point_range = float(values.get("range", math.sqrt(lx * lx + ly * ly + lz * lz)))
        layer_value = values.get("layer", values.get("ring", -1))
        echo_value = values.get("echo", -1)
        sample = Sample(
            x=bx,
            y=by,
            z=bz,
            intensity=intensity,
            range=point_range,
            layer=int(round(float(layer_value))) if layer_value is not None else -1,
            echo=int(round(float(echo_value))) if echo_value is not None else -1,
            reflector=bool(float(values.get("reflector", 0.0)) > 0.5),
        )
        if not all(math.isfinite(v) for v in (sample.x, sample.y, sample.z, sample.intensity, sample.range)):
            continue
        if not pass_layer_echo(sample, args):
            continue
        if sample.range < args.range_min_m or sample.range > args.range_max_m:
            continue
        if sample.x < args.base_min_x_m or sample.x > args.base_max_x_m:
            continue
        if abs(sample.y) > args.base_max_abs_y_m:
            continue
        if abs(sample.z - args.ground_z_m) > args.ground_z_tolerance_m:
            continue
        samples.append(sample)
    return samples


def select_candidates(samples: list[Sample], params: SweepParams, args: argparse.Namespace) -> list[Sample]:
    global_stats = RunningStats()
    bin_stats: dict[tuple[int, int], RunningStats] = defaultdict(RunningStats)
    for sample in samples:
        global_stats.add(sample.intensity)
        bin_stats[stats_key(sample, params.adaptive_range_bin_m, args.normalize_by_layer)].add(sample.intensity)

    candidates: list[Sample] = []
    for sample in samples:
        local_stats = bin_stats[stats_key(sample, params.adaptive_range_bin_m, args.normalize_by_layer)]
        stats = local_stats if local_stats.count >= args.adaptive_min_samples else global_stats
        threshold = stats.mean + max(
            params.adaptive_min_delta,
            params.adaptive_stddev_multiplier * stats.stddev,
        )
        threshold = max(threshold, params.min_intensity)
        if sample.intensity >= threshold:
            candidates.append(sample)
    return candidates


def grid_key(x: int, y: int) -> tuple[int, int]:
    return x, y


def cluster_candidates(candidates: list[Sample], link_distance: float) -> list[list[int]]:
    grid: dict[tuple[int, int], list[int]] = defaultdict(list)
    for index, sample in enumerate(candidates):
        gx = int(math.floor(sample.x / link_distance))
        gy = int(math.floor(sample.y / link_distance))
        grid[grid_key(gx, gy)].append(index)

    visited = [False] * len(candidates)
    clusters: list[list[int]] = []
    eps_sq = link_distance * link_distance
    for start in range(len(candidates)):
        if visited[start]:
            continue
        visited[start] = True
        cluster: list[int] = []
        queue = deque([start])
        while queue:
            index = queue.popleft()
            cluster.append(index)
            gx = int(math.floor(candidates[index].x / link_distance))
            gy = int(math.floor(candidates[index].y / link_distance))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    for nb in grid.get(grid_key(gx + dx, gy + dy), []):
                        if visited[nb]:
                            continue
                        ddx = candidates[nb].x - candidates[index].x
                        ddy = candidates[nb].y - candidates[index].y
                        if ddx * ddx + ddy * ddy <= eps_sq:
                            visited[nb] = True
                            queue.append(nb)
        clusters.append(cluster)
    return clusters


def compute_geometry(candidates: list[Sample], cluster: list[int], min_points: int) -> ClusterGeometry | None:
    if len(cluster) < min_points:
        return None
    cx = sum(candidates[i].x for i in cluster) / len(cluster)
    cy = sum(candidates[i].y for i in cluster) / len(cluster)
    mean_z = sum(candidates[i].z for i in cluster) / len(cluster)
    if len(cluster) == 1:
        return None
    cov_xx = 0.0
    cov_xy = 0.0
    cov_yy = 0.0
    for i in cluster:
        dx = candidates[i].x - cx
        dy = candidates[i].y - cy
        cov_xx += dx * dx
        cov_xy += dx * dy
        cov_yy += dy * dy
    denom = max(1, len(cluster) - 1)
    cov_xx /= denom
    cov_xy /= denom
    cov_yy /= denom

    trace = cov_xx + cov_yy
    disc = math.sqrt(max(0.0, (cov_xx - cov_yy) ** 2 + 4.0 * cov_xy * cov_xy))
    major_lambda = 0.5 * (trace + disc)
    if abs(cov_xy) > 1e-9:
        major_x = major_lambda - cov_yy
        major_y = cov_xy
    elif cov_xx >= cov_yy:
        major_x, major_y = 1.0, 0.0
    else:
        major_x, major_y = 0.0, 1.0
    norm = math.hypot(major_x, major_y)
    if norm < 1e-9:
        return None
    major_x /= norm
    major_y /= norm
    minor_x = -major_y
    minor_y = major_x

    major_vals = []
    minor_vals = []
    for i in cluster:
        dx = candidates[i].x - cx
        dy = candidates[i].y - cy
        major_vals.append(dx * major_x + dy * major_y)
        minor_vals.append(dx * minor_x + dy * minor_y)

    return ClusterGeometry(
        cx=cx,
        cy=cy,
        major_x=major_x,
        major_y=major_y,
        minor_x=minor_x,
        minor_y=minor_y,
        major_min=min(major_vals),
        major_max=max(major_vals),
        minor_min=min(minor_vals),
        minor_max=max(minor_vals),
        mean_z=mean_z,
    )


def accepts_geometry(geom: ClusterGeometry, args: argparse.Namespace) -> bool:
    return (
        geom.length >= args.cluster_min_length_m
        and geom.width <= args.cluster_max_width_m
        and geom.aspect >= args.cluster_min_aspect_ratio
    )


def in_window(rel_time: float, window: list[float] | None) -> bool:
    return bool(window) and window[0] <= rel_time <= window[1]


def evaluate_params(
    cloud_samples: list[tuple[float, list[Sample]]],
    params: SweepParams,
    args: argparse.Namespace,
) -> SweepResult:
    result = SweepResult(params=params, cluster_rows=[])
    widths = []
    for cloud_index, (rel_time, samples) in enumerate(cloud_samples):
        result.clouds += 1
        result.ground_samples += len(samples)
        candidates = select_candidates(samples, params, args)
        result.candidates += len(candidates)
        clusters = cluster_candidates(candidates, args.cluster_link_distance_m)
        result.clusters += len(clusters)
        cloud_accepted_length = 0.0
        for cluster in clusters:
            geom = compute_geometry(candidates, cluster, args.cluster_min_points)
            if geom and accepts_geometry(geom, args):
                result.accepted_clusters += 1
                result.accepted_points += len(cluster)
                result.accepted_length_m += geom.length
                cloud_accepted_length += geom.length
                result.longest_cluster_m = max(result.longest_cluster_m, geom.length)
                widths.append(geom.width)
                if len(result.cluster_rows) < args.max_cluster_rows:
                    intensities = [candidates[i].intensity for i in cluster]
                    result.cluster_rows.append(
                        {
                            "cloud_index": cloud_index,
                            "min_intensity": params.min_intensity,
                            "adaptive_range_bin_m": params.adaptive_range_bin_m,
                            "adaptive_min_delta": params.adaptive_min_delta,
                            "adaptive_stddev_multiplier": params.adaptive_stddev_multiplier,
                            "points": len(cluster),
                            "centroid_x": geom.cx,
                            "centroid_y": geom.cy,
                            "mean_z": geom.mean_z,
                            "length_m": geom.length,
                            "width_m": geom.width,
                            "aspect": geom.aspect,
                            "mean_intensity": sum(intensities) / len(intensities),
                            "max_intensity": max(intensities),
                        }
                    )
            else:
                result.rejected_clusters += 1
        if in_window(rel_time, args.positive_window):
            result.positive_length_m += cloud_accepted_length
        if in_window(rel_time, args.negative_window):
            result.negative_length_m += cloud_accepted_length
    if widths:
        widths_sorted = sorted(widths)
        result.median_width_m = widths_sorted[len(widths_sorted) // 2]
    return result


def load_cloud_samples(
    bag_path: str,
    topic_types: dict[str, str],
    graph: TransformGraph,
    args: argparse.Namespace,
) -> list[tuple[float, list[Sample]]]:
    if args.topic not in topic_types:
        raise SystemExit(f"{args.topic} not found in bag. Found: {sorted(topic_types)}")
    msg_type = get_message(topic_types[args.topic])
    reader = open_reader(bag_path)
    cloud_samples: list[tuple[float, list[Sample]]] = []
    seen_clouds = 0
    first_cloud_stamp_ns: int | None = None
    failed_tf_frames: set[str] = set()
    while reader.has_next():
        topic, data, _stamp_ns = reader.read_next()
        if topic != args.topic:
            continue
        seen_clouds += 1
        if args.cloud_stride > 1 and (seen_clouds - 1) % args.cloud_stride:
            continue
        if args.max_clouds and len(cloud_samples) >= args.max_clouds:
            break
        if first_cloud_stamp_ns is None:
            first_cloud_stamp_ns = _stamp_ns
        rel_time = (_stamp_ns - first_cloud_stamp_ns) / 1e9
        msg = deserialize_message(data, msg_type)
        source_frame = msg.header.frame_id.strip("/")
        transform = graph.lookup(args.base_frame, source_frame)
        if transform is None and source_frame == "lidar_footprint" and args.fallback_shogi_lidar_tf:
            # base_link -> lidar_footprint from shogi.urdf, inverted as
            # lidar_footprint -> base_link. Roll is pi, so y/z sign flip.
            transform = [
                [1.0, 0.0, 0.0, 0.659800],
                [0.0, -1.0, 0.0, 0.000105],
                [0.0, 0.0, -1.0, 0.205680],
                [0.0, 0.0, 0.0, 1.0],
            ]
        if transform is None:
            failed_tf_frames.add(source_frame)
            continue
        cloud_samples.append((rel_time, extract_samples(msg, transform, args)))
    if failed_tf_frames:
        print(f"warning: skipped clouds with no TF to {args.base_frame}: {sorted(failed_tf_frames)}")
    if not cloud_samples:
        raise SystemExit("No analyzable cloud samples found.")
    return cloud_samples


def write_csv(path: str, rows: Iterable[dict]) -> None:
    rows = list(rows)
    if not rows:
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/cloud_all_fields_fullframe")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--intensity-field", default="i")
    parser.add_argument("--cloud-stride", type=int, default=10)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--max-clouds", type=int, default=120)
    parser.add_argument(
        "--no-fallback-shogi-lidar-tf",
        dest="fallback_shogi_lidar_tf",
        action="store_false",
        help="Disable the built-in shogi.urdf lidar_footprint -> base_link fallback.",
    )
    parser.set_defaults(fallback_shogi_lidar_tf=True)

    parser.add_argument("--range-min-m", type=float, default=0.20)
    parser.add_argument("--range-max-m", type=float, default=6.0)
    parser.add_argument("--base-min-x-m", type=float, default=-0.10)
    parser.add_argument("--base-max-x-m", type=float, default=5.0)
    parser.add_argument("--base-max-abs-y-m", type=float, default=3.0)
    parser.add_argument("--ground-z-m", type=float, default=-0.11)
    parser.add_argument("--ground-z-tolerance-m", type=float, default=0.08)
    parser.add_argument("--layer-min", type=int, default=-1)
    parser.add_argument("--layer-max", type=int, default=-1)
    parser.add_argument("--echo-filter", type=int, default=-1)

    parser.add_argument(
        "--adaptive-range-bin-m",
        type=float,
        default=0.25,
        help="Single range-bin width used when --adaptive-range-bin-values is empty.",
    )
    parser.add_argument(
        "--adaptive-range-bin-values",
        default="0.15,0.25,0.50",
        help="Comma-separated adaptive range-bin widths to sweep in meters.",
    )
    parser.add_argument("--adaptive-min-samples", type=int, default=20)
    parser.add_argument("--normalize-by-layer", action="store_true", default=True)
    parser.add_argument(
        "--min-intensity-values",
        default="0,5000,10000,20000,30000,40000",
    )
    parser.add_argument(
        "--adaptive-min-delta-values",
        default="100,250,500,900,1500",
    )
    parser.add_argument(
        "--adaptive-stddev-multiplier-values",
        default="0.5,1.0,1.5,2.0",
    )

    parser.add_argument("--cluster-link-distance-m", type=float, default=0.20)
    parser.add_argument("--cluster-min-points", type=int, default=3)
    parser.add_argument("--cluster-min-length-m", type=float, default=0.20)
    parser.add_argument("--cluster-max-width-m", type=float, default=0.22)
    parser.add_argument("--cluster-min-aspect-ratio", type=float, default=2.5)
    parser.add_argument("--top", type=int, default=15)
    parser.add_argument("--max-cluster-rows", type=int, default=200)
    parser.add_argument(
        "--positive-window",
        nargs=2,
        type=float,
        metavar=("START_SEC", "END_SEC"),
        default=None,
        help="Relative bag-time window where tape is visible; boosts rankings.",
    )
    parser.add_argument(
        "--negative-window",
        nargs=2,
        type=float,
        metavar=("START_SEC", "END_SEC"),
        default=None,
        help="Relative bag-time window with non-line ground; penalizes detections.",
    )
    parser.add_argument("--csv", default="")
    parser.add_argument("--clusters-csv", default="")
    args = parser.parse_args()

    args.cloud_stride = max(1, args.cloud_stride)
    args.point_stride = max(1, args.point_stride)
    args.adaptive_range_bin_m = max(0.05, args.adaptive_range_bin_m)
    args.adaptive_min_samples = max(1, args.adaptive_min_samples)
    args.cluster_link_distance_m = max(0.02, args.cluster_link_distance_m)
    args.cluster_min_points = max(1, args.cluster_min_points)

    topic_types = get_topic_types(args.bag)
    graph = build_tf_graph(args.bag, topic_types)
    cloud_samples = load_cloud_samples(args.bag, topic_types, graph, args)

    min_intensities = parse_float_list(args.min_intensity_values)
    range_bins = parse_float_list(args.adaptive_range_bin_values)
    if not range_bins:
        range_bins = [args.adaptive_range_bin_m]
    range_bins = [max(0.05, value) for value in range_bins]
    deltas = parse_float_list(args.adaptive_min_delta_values)
    multipliers = parse_float_list(args.adaptive_stddev_multiplier_values)
    sweep_params = [
        SweepParams(mi, range_bin, delta, mult)
        for mi in min_intensities
        for range_bin in range_bins
        for delta in deltas
        for mult in multipliers
    ]

    results = [evaluate_params(cloud_samples, params, args) for params in sweep_params]
    results.sort(key=lambda item: item.score(), reverse=True)

    print("LiDAR RSSI line sweep")
    print(f"bag: {args.bag}")
    print(f"topic: {args.topic}")
    print(
        f"clouds analyzed: {len(cloud_samples)} "
        f"(cloud_stride={args.cloud_stride}, max_clouds={args.max_clouds})"
    )
    print(f"parameter combinations: {len(results)}")
    print()
    if args.positive_window or args.negative_window:
        print(
            "rank  min_i  bin_m  delta  stdx  ground  cand  cand%  acc_cl  "
            "rej_cl  pos_len  neg_len  acc_len  longest  med_width"
        )
    else:
        print(
            "rank  min_i  bin_m  delta  stdx  ground  cand  cand%  acc_cl  "
            "rej_cl  acc_len  longest  med_width"
        )
    for rank, result in enumerate(results[: args.top], start=1):
        cand_frac = 100.0 * result.candidates / max(1, result.ground_samples)
        common = (
            f"{rank:>4} "
            f"{result.params.min_intensity:>6.0f} "
            f"{result.params.adaptive_range_bin_m:>5.2f} "
            f"{result.params.adaptive_min_delta:>6.0f} "
            f"{result.params.adaptive_stddev_multiplier:>5.1f} "
            f"{result.ground_samples:>7} "
            f"{result.candidates:>6} "
            f"{cand_frac:>5.2f} "
            f"{result.accepted_clusters:>6} "
            f"{result.rejected_clusters:>6} "
        )
        if args.positive_window or args.negative_window:
            print(
                common
                + f"{result.positive_length_m:>7.2f} "
                + f"{result.negative_length_m:>7.2f} "
                + f"{result.accepted_length_m:>7.2f} "
                + f"{result.longest_cluster_m:>7.2f} "
                + f"{result.median_width_m:>9.3f}"
            )
        else:
            print(
                common
                + f"{result.accepted_length_m:>7.2f} "
                + f"{result.longest_cluster_m:>7.2f} "
                + f"{result.median_width_m:>9.3f}"
            )

    if args.csv:
        rows = []
        for result in results:
            rows.append(
                {
                    "min_intensity": result.params.min_intensity,
                    "adaptive_range_bin_m": result.params.adaptive_range_bin_m,
                    "adaptive_min_delta": result.params.adaptive_min_delta,
                    "adaptive_stddev_multiplier": result.params.adaptive_stddev_multiplier,
                    "clouds": result.clouds,
                    "ground_samples": result.ground_samples,
                    "candidates": result.candidates,
                    "candidate_fraction": result.candidates / max(1, result.ground_samples),
                    "clusters": result.clusters,
                    "accepted_clusters": result.accepted_clusters,
                    "rejected_clusters": result.rejected_clusters,
                    "accepted_points": result.accepted_points,
                    "accepted_length_m": result.accepted_length_m,
                    "positive_length_m": result.positive_length_m,
                    "negative_length_m": result.negative_length_m,
                    "longest_cluster_m": result.longest_cluster_m,
                    "median_width_m": result.median_width_m,
                }
            )
        write_csv(args.csv, rows)
        print(f"\nwrote sweep CSV: {args.csv}")

    if args.clusters_csv and results:
        write_csv(args.clusters_csv, results[0].cluster_rows or [])
        print(f"wrote top-cluster CSV: {args.clusters_csv}")

    best = results[0]
    print()
    print("Suggested live-test override:")
    print(
        "  ros2 run autonav_detection lidar_line_detector --ros-args "
        "--params-file $(ros2 pkg prefix autonav_detection)/share/autonav_detection/config/lidar_line_detector_rssi.yaml "
        f"-p min_intensity:={best.params.min_intensity:.0f} "
        f"-p adaptive_range_bin_m:={best.params.adaptive_range_bin_m:.2f} "
        f"-p adaptive_min_delta:={best.params.adaptive_min_delta:.0f} "
        f"-p adaptive_stddev_multiplier:={best.params.adaptive_stddev_multiplier:.2f}"
    )


if __name__ == "__main__":
    main()
