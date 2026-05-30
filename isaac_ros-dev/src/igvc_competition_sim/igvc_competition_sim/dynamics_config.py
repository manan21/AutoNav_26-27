from __future__ import annotations

from pathlib import Path
from typing import Any

from .course import PACKAGE_ROOT, load_yaml


MPH_TO_MPS = 0.44704


def package_config_path(filename: str) -> Path:
    source_tree = PACKAGE_ROOT / "config" / filename
    if source_tree.is_file():
        return source_tree
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_tree
    return Path(get_package_share_directory("igvc_competition_sim")) / "config" / filename


def default_calibration_path() -> Path:
    return package_config_path("dynamics_calibration.yaml")


def default_replay_profiles_path() -> Path:
    return package_config_path("dynamics_replay_profiles.yaml")


def load_config(path: str | Path | None) -> dict[str, Any]:
    if path is None or str(path).strip() in ("", "__auto__"):
        return load_yaml(default_calibration_path())
    return load_yaml(Path(path).expanduser().resolve())


def load_replay_profiles(path: str | Path | None) -> dict[str, Any]:
    if path is None or str(path).strip() in ("", "__auto__"):
        return load_yaml(default_replay_profiles_path())
    return load_yaml(Path(path).expanduser().resolve())


def as_float(raw: Any, default: float) -> float:
    if raw is None:
        return default
    return float(raw)


def as_bool(raw: Any, default: bool) -> bool:
    if raw is None:
        return default
    if isinstance(raw, bool):
        return raw
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def segment_linear_mps(segment: dict[str, Any]) -> float:
    if "linear_mps" in segment:
        return float(segment["linear_mps"])
    if "linear_mph" in segment:
        return float(segment["linear_mph"]) * MPH_TO_MPS
    return 0.0


def segment_angular_radps(segment: dict[str, Any]) -> float:
    return float(segment.get("angular_radps", 0.0))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
