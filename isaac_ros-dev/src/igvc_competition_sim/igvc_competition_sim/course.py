from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EARTH_RADIUS_M = 6378137.0


def _default_course_config() -> Path:
    source_tree = PACKAGE_ROOT / "config" / "igvc_competition_compact.yaml"
    if source_tree.is_file():
        return source_tree
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_tree
    share_tree = (
        Path(get_package_share_directory("igvc_competition_sim"))
        / "config"
        / "igvc_competition_compact.yaml"
    )
    return share_tree


DEFAULT_COURSE_CONFIG = _default_course_config()


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class TapeSegment:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]
    width_m: float


@dataclass(frozen=True)
class Obstacle:
    name: str
    kind: str
    center: tuple[float, float]
    radius_m: float
    height_m: float


@dataclass(frozen=True)
class Pothole:
    name: str
    center: tuple[float, float]
    radius_m: float


@dataclass(frozen=True)
class Ramp:
    name: str
    start_x_m: float
    end_x_m: float
    center_y_m: float
    width_m: float
    rise_m: float


@dataclass(frozen=True)
class MissionWaypoint:
    label: str
    kind: str
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True)
class AnalysisStation:
    label: str
    x_m: float
    y_min_m: float
    y_max_m: float


@dataclass(frozen=True)
class RobotSpec:
    base_link_to_nav_center_m: float
    lidar_x_from_base_link_m: float
    lidar_z_from_base_link_m: float
    base_link_height_above_ground_m: float
    gps_x_from_base_link_m: float
    gps_y_from_base_link_m: float
    gps_z_from_base_link_m: float
    wheel_track_m: float
    wheel_radius_m: float
    physical_half_length_m: float
    physical_half_width_m: float
    footprint_padding_m: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    cmd_latency_s: float
    linear_time_constant_s: float
    angular_time_constant_s: float


@dataclass(frozen=True)
class SpeedCheck:
    start_x_m: float
    end_distance_m: float
    minimum_average_mps: float
    maximum_speed_mps: float
    blocking_stop_s: float


@dataclass(frozen=True)
class Course:
    course_id: str
    description: str
    config_path: Path
    datum_latitude_deg: float
    datum_longitude_deg: float
    datum_altitude_m: float
    start: Pose2D
    finish: tuple[float, float, float]
    robot: RobotSpec
    tapes: tuple[TapeSegment, ...]
    obstacles: tuple[Obstacle, ...]
    potholes: tuple[Pothole, ...]
    ramps: tuple[Ramp, ...]
    mission_waypoints: tuple[MissionWaypoint, ...]
    analysis_stations: tuple[AnalysisStation, ...]
    speed_check: SpeedCheck


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        return _load_limited_yaml(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Course config {path} did not load as a mapping")
    return data


def _strip_comment(line: str) -> str:
    in_single = False
    in_double = False
    for idx, ch in enumerate(line):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == "#" and not in_single and not in_double:
            return line[:idx]
    return line


def _limited_yaml_lines(path: Path) -> list[tuple[int, str]]:
    lines: list[tuple[int, str]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        without_comment = _strip_comment(raw).rstrip()
        if not without_comment.strip():
            continue
        indent = len(without_comment) - len(without_comment.lstrip(" "))
        lines.append((indent, without_comment.strip()))
    return lines


def _split_top_level(raw: str, delimiter: str) -> list[str]:
    parts: list[str] = []
    start = 0
    depth = 0
    in_single = False
    in_double = False
    for idx, ch in enumerate(raw):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif not in_single and not in_double:
            if ch in "[{(":
                depth += 1
            elif ch in "]})":
                depth -= 1
            elif ch == delimiter and depth == 0:
                parts.append(raw[start:idx].strip())
                start = idx + 1
    parts.append(raw[start:].strip())
    return [part for part in parts if part]


def _parse_scalar(raw: str) -> Any:
    value = raw.strip()
    if value == "":
        return ""
    if value.startswith("{") and value.endswith("}"):
        result: dict[str, Any] = {}
        inner = value[1:-1].strip()
        if not inner:
            return result
        for item in _split_top_level(inner, ","):
            key, val = item.split(":", 1)
            result[key.strip()] = _parse_scalar(val)
        return result
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        return [_parse_scalar(item) for item in _split_top_level(inner, ",")]
    if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    lower = value.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _parse_mapping(lines: list[tuple[int, str]],
                   idx: int,
                   indent: int) -> tuple[dict[str, Any], int]:
    out: dict[str, Any] = {}
    while idx < len(lines):
        line_indent, text = lines[idx]
        if line_indent < indent:
            break
        if line_indent > indent:
            raise ValueError(f"Unexpected indentation near: {text}")
        if text.startswith("- "):
            break
        if ":" not in text:
            raise ValueError(f"Expected mapping entry near: {text}")
        key, raw_value = text.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        idx += 1
        if raw_value:
            out[key] = _parse_scalar(raw_value)
            continue
        if idx >= len(lines) or lines[idx][0] <= indent:
            out[key] = {}
            continue
        child_indent = lines[idx][0]
        if lines[idx][1].startswith("- "):
            out[key], idx = _parse_list(lines, idx, child_indent)
        else:
            out[key], idx = _parse_mapping(lines, idx, child_indent)
    return out, idx


def _parse_list(lines: list[tuple[int, str]],
                idx: int,
                indent: int) -> tuple[list[Any], int]:
    out: list[Any] = []
    while idx < len(lines):
        line_indent, text = lines[idx]
        if line_indent < indent:
            break
        if line_indent != indent or not text.startswith("- "):
            break
        item_text = text[2:].strip()
        idx += 1
        if item_text.startswith("{"):
            out.append(_parse_scalar(item_text))
            continue
        if item_text and ":" in item_text:
            key, raw_value = item_text.split(":", 1)
            item: dict[str, Any] = {
                key.strip(): _parse_scalar(raw_value.strip())
            }
            if idx < len(lines) and lines[idx][0] > indent:
                more, idx = _parse_mapping(lines, idx, lines[idx][0])
                item.update(more)
            out.append(item)
            continue
        if idx < len(lines) and lines[idx][0] > indent:
            if lines[idx][1].startswith("- "):
                value, idx = _parse_list(lines, idx, lines[idx][0])
            else:
                value, idx = _parse_mapping(lines, idx, lines[idx][0])
            out.append(value)
        else:
            out.append(_parse_scalar(item_text))
    return out, idx


def _load_limited_yaml(path: Path) -> dict[str, Any]:
    lines = _limited_yaml_lines(path)
    if not lines:
        return {}
    data, idx = _parse_mapping(lines, 0, lines[0][0])
    if idx != len(lines):
        raise ValueError(f"Could not parse all of {path}")
    return data


def _as_path(path: str | Path | None) -> Path:
    if path is None or str(path).strip() in ("", "__auto__"):
        return DEFAULT_COURSE_CONFIG
    return Path(path).expanduser().resolve()


def _point(raw: dict[str, Any]) -> tuple[float, float]:
    return float(raw["x_m"]), float(raw["y_m"])


def _station_normal(points: list[tuple[float, float, float]],
                    idx: int) -> tuple[float, float]:
    if idx == 0:
        dx = points[1][0] - points[0][0]
        dy = points[1][1] - points[0][1]
    elif idx == len(points) - 1:
        dx = points[-1][0] - points[-2][0]
        dy = points[-1][1] - points[-2][1]
    else:
        dx = points[idx + 1][0] - points[idx - 1][0]
        dy = points[idx + 1][1] - points[idx - 1][1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 0.0, 1.0
    return -dy / length, dx / length


def _boundary_tapes(data: dict[str, Any],
                    tape_width_m: float) -> list[TapeSegment]:
    raw_centerline = data.get("centerline", [])
    points = [
        (float(p["x_m"]), float(p["y_m"]), float(p["width_m"]))
        for p in raw_centerline
    ]
    if len(points) < 2:
        raise ValueError("centerline must contain at least two stations")

    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    for idx, (x, y, width) in enumerate(points):
        nx, ny = _station_normal(points, idx)
        half = 0.5 * width
        left.append((x + nx * half, y + ny * half))
        right.append((x - nx * half, y - ny * half))

    tapes: list[TapeSegment] = []
    for idx in range(len(points) - 1):
        tapes.append(TapeSegment(
            name=f"left_boundary_{idx}",
            start=left[idx],
            end=left[idx + 1],
            width_m=tape_width_m,
        ))
        tapes.append(TapeSegment(
            name=f"right_boundary_{idx}",
            start=right[idx],
            end=right[idx + 1],
            width_m=tape_width_m,
        ))
    return tapes


def _dashed_segments(raw: dict[str, Any],
                     default_width_m: float) -> list[TapeSegment]:
    start = _point(raw["start"])
    end = _point(raw["end"])
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return []
    ux = dx / length
    uy = dy / length
    dash = float(raw.get("dash_length_m", 0.75))
    gap = float(raw.get("gap_length_m", 0.45))
    width = float(raw.get("width_m", default_width_m))
    out: list[TapeSegment] = []
    offset = 0.0
    idx = 0
    while offset < length:
        seg_len = min(dash, length - offset)
        if seg_len > 1e-6:
            out.append(TapeSegment(
                name=f"{raw.get('name', 'dash')}_{idx}",
                start=(start[0] + ux * offset, start[1] + uy * offset),
                end=(start[0] + ux * (offset + seg_len),
                     start[1] + uy * (offset + seg_len)),
                width_m=width,
            ))
            idx += 1
        offset += dash + gap
    return out


def load_course(path: str | Path | None = None) -> Course:
    config_path = _as_path(path)
    data = load_yaml(config_path)
    tape_width = float(data.get("tape_width_m", 0.12))
    tapes = _boundary_tapes(data, tape_width)
    for raw in data.get("internal_lines", []):
        tapes.append(TapeSegment(
            name=str(raw.get("name", "internal_line")),
            start=_point(raw["start"]),
            end=_point(raw["end"]),
            width_m=float(raw.get("width_m", tape_width)),
        ))
    for raw in data.get("dashed_lines", []):
        tapes.extend(_dashed_segments(raw, tape_width))

    datum = data["datum"]
    start = data["start"]
    finish = data["finish"]
    robot = data["robot"]
    speed_check = data["speed_check"]

    return Course(
        course_id=str(data.get("course_id", config_path.stem)),
        description=str(data.get("description", "")),
        config_path=config_path,
        datum_latitude_deg=float(datum["latitude_deg"]),
        datum_longitude_deg=float(datum["longitude_deg"]),
        datum_altitude_m=float(datum.get("altitude_m", 0.0)),
        start=Pose2D(
            x=float(start.get("x_m", 0.0)),
            y=float(start.get("y_m", 0.0)),
            yaw=float(start.get("yaw_rad", 0.0)),
        ),
        finish=(
            float(finish["x_m"]),
            float(finish["y_m"]),
            float(finish.get("radius_m", 1.0)),
        ),
        robot=RobotSpec(**{key: float(value) for key, value in robot.items()}),
        tapes=tuple(tapes),
        obstacles=tuple(
            Obstacle(
                name=str(raw["name"]),
                kind=str(raw.get("type", "barrel")),
                center=(float(raw["x_m"]), float(raw["y_m"])),
                radius_m=float(raw["radius_m"]),
                height_m=float(raw["height_m"]),
            )
            for raw in data.get("obstacles", [])
        ),
        potholes=tuple(
            Pothole(
                name=str(raw["name"]),
                center=(float(raw["x_m"]), float(raw["y_m"])),
                radius_m=float(raw["radius_m"]),
            )
            for raw in data.get("potholes", [])
        ),
        ramps=tuple(
            Ramp(
                name=str(raw["name"]),
                start_x_m=float(raw["start_x_m"]),
                end_x_m=float(raw["end_x_m"]),
                center_y_m=float(raw["center_y_m"]),
                width_m=float(raw["width_m"]),
                rise_m=float(raw["rise_m"]),
            )
            for raw in data.get("ramps", [])
        ),
        mission_waypoints=tuple(
            MissionWaypoint(
                label=str(raw["label"]),
                kind=str(raw["type"]),
                x_m=float(raw["x_m"]),
                y_m=float(raw["y_m"]),
                radius_m=float(raw.get("radius_m", 1.0)),
            )
            for raw in data.get("mission_waypoints", [])
        ),
        analysis_stations=tuple(
            AnalysisStation(
                label=str(raw["label"]),
                x_m=float(raw["x_m"]),
                y_min_m=float(raw["y_min_m"]),
                y_max_m=float(raw["y_max_m"]),
            )
            for raw in data.get("analysis_stations", [])
        ),
        speed_check=SpeedCheck(
            start_x_m=float(speed_check["start_x_m"]),
            end_distance_m=float(speed_check["end_distance_m"]),
            minimum_average_mps=float(speed_check["minimum_average_mps"]),
            maximum_speed_mps=float(speed_check["maximum_speed_mps"]),
            blocking_stop_s=float(speed_check["blocking_stop_s"]),
        ),
    )


def local_to_latlon(x_m: float,
                    y_m: float,
                    datum_lat_deg: float,
                    datum_lon_deg: float) -> tuple[float, float]:
    datum_lat_rad = math.radians(datum_lat_deg)
    lat = datum_lat_deg + math.degrees(y_m / EARTH_RADIUS_M)
    lon = datum_lon_deg + math.degrees(
        x_m / (EARTH_RADIUS_M * math.cos(datum_lat_rad)))
    return lat, lon


def latlon_to_local(lat_deg: float,
                    lon_deg: float,
                    datum_lat_deg: float,
                    datum_lon_deg: float) -> tuple[float, float]:
    datum_lat_rad = math.radians(datum_lat_deg)
    x = math.radians(lon_deg - datum_lon_deg) * EARTH_RADIUS_M * math.cos(
        datum_lat_rad)
    y = math.radians(lat_deg - datum_lat_deg) * EARTH_RADIUS_M
    return x, y


def iter_course_points(course: Course) -> Iterable[tuple[float, float]]:
    yield course.start.x, course.start.y
    yield course.finish[0], course.finish[1]
    for tape in course.tapes:
        yield tape.start
        yield tape.end
    for obstacle in course.obstacles:
        yield obstacle.center
    for pothole in course.potholes:
        yield pothole.center
    for ramp in course.ramps:
        yield ramp.start_x_m, ramp.center_y_m
        yield ramp.end_x_m, ramp.center_y_m
    for waypoint in course.mission_waypoints:
        yield waypoint.x_m, waypoint.y_m


def course_bounds(course: Course, margin_m: float = 4.0
                  ) -> tuple[float, float, float, float]:
    points = tuple(iter_course_points(course))
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (
        math.floor(min(xs) - margin_m),
        math.floor(min(ys) - margin_m),
        math.ceil(max(xs) + margin_m),
        math.ceil(max(ys) + margin_m),
    )
