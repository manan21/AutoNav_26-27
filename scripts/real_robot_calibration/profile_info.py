#!/usr/bin/env python3
"""Profile helpers for the real robot calibration scripts.

profiles.yaml is intentionally strict JSON. JSON is valid YAML, and using it
keeps this suite free of a PyYAML dependency on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


MPH_TO_MPS = 0.44704


def load_config(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path}: invalid strict-JSON profiles.yaml: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("profiles"), dict):
        raise SystemExit(f"{path}: expected top-level object with a profiles object")
    return data


def get_profile(config: dict[str, Any], name: str) -> dict[str, Any]:
    profiles = config["profiles"]
    if name not in profiles:
        available = ", ".join(sorted(profiles))
        raise SystemExit(f"Unknown profile '{name}'. Available profiles: {available}")
    profile = profiles[name]
    if not isinstance(profile, dict):
        raise SystemExit(f"Profile '{name}' must be an object")
    return profile


def iter_segments(profile: dict[str, Any]) -> list[dict[str, Any]]:
    segments = profile.get("segments", [])
    if segments is None:
        return []
    if not isinstance(segments, list):
        raise SystemExit("segments must be a list")
    for segment in segments:
        if not isinstance(segment, dict):
            raise SystemExit("each segment must be an object")
    return segments


def segment_velocity(segment: dict[str, Any]) -> tuple[float, float]:
    has_mph = "linear_mph" in segment
    has_mps = "linear_mps" in segment
    if has_mph and has_mps:
        raise SystemExit(f"{segment.get('label', '<unnamed>')}: use either linear_mph or linear_mps, not both")
    linear = float(segment.get("linear_mps", 0.0))
    if has_mph:
        linear = float(segment["linear_mph"]) * MPH_TO_MPS
    angular = float(segment.get("angular_radps", 0.0))
    return linear, angular


def duration_seconds(profile: dict[str, Any]) -> float | None:
    if profile.get("command_mode") != "scripted":
        return None
    total = 0.0
    for segment in iter_segments(profile):
        total += float(segment.get("duration_s", 0.0))
    return total


def max_abs_speed_mps(profile: dict[str, Any]) -> float:
    max_speed = 0.0
    for segment in iter_segments(profile):
        linear, _angular = segment_velocity(segment)
        max_speed = max(max_speed, abs(linear))
    return max_speed


def validate_profile(name: str, config: dict[str, Any]) -> None:
    profile = get_profile(config, name)
    bag_profile = profile.get("bag_profile")
    if not isinstance(bag_profile, str) or not bag_profile:
        raise SystemExit(f"{name}: bag_profile is required")
    command_mode = profile.get("command_mode", "none")
    if command_mode not in {"none", "scripted"}:
        raise SystemExit(f"{name}: command_mode must be 'none' or 'scripted'")
    if command_mode == "scripted" and not iter_segments(profile):
        raise SystemExit(f"{name}: scripted profiles require non-empty segments")
    for segment in iter_segments(profile):
        if "duration_s" not in segment:
            raise SystemExit(f"{name}: segment {segment.get('label', '<unnamed>')} missing duration_s")
        duration = float(segment["duration_s"])
        if not math.isfinite(duration) or duration < 0.0:
            raise SystemExit(f"{name}: invalid segment duration {duration}")
        linear, angular = segment_velocity(segment)
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise SystemExit(f"{name}: non-finite velocity in segment {segment.get('label', '<unnamed>')}")


def git_value(args: list[str]) -> str:
    try:
        return subprocess.check_output(
            ["git", *args],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return "unknown"


def topic_lines(path: Path) -> list[str]:
    topics: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            topics.append(line)
    return topics


def write_metadata(
    *,
    output: Path,
    profiles_path: Path,
    profile_name: str,
    run_name: str,
    bag_path: str,
    topic_file: Path,
    allow_high_speed: bool,
    raw_lidar: bool,
    argv_text: str,
) -> None:
    config = load_config(profiles_path)
    validate_profile(profile_name, config)
    profile = get_profile(config, profile_name)
    duration = duration_seconds(profile)
    topics = topic_lines(topic_file)

    lines = [
        f"run_name: {run_name}",
        f"profile: {profile_name}",
        f"description: {profile.get('description', '')}",
        f"bag_profile: {profile.get('bag_profile', '')}",
        f"bag_path: {bag_path}",
        f"topic_file: {topic_file}",
        f"topic_count: {len(topics)}",
        f"raw_lidar_enabled: {raw_lidar}",
        f"allow_high_speed: {allow_high_speed}",
        f"command_mode: {profile.get('command_mode', 'none')}",
        f"wait_for_auto: {bool(profile.get('wait_for_auto', False))}",
        f"record_until_interrupt: {bool(profile.get('record_until_interrupt', False))}",
        f"expected_scripted_duration_s: {duration if duration is not None else 'manual_stop'}",
        f"max_abs_command_speed_mps: {max_abs_speed_mps(profile):.6f}",
        f"max_abs_command_speed_mph: {max_abs_speed_mps(profile) / MPH_TO_MPS:.3f}",
        f"operator_notes: {profile.get('operator_notes', '')}",
        f"hostname: {socket.gethostname()}",
        f"created_utc: {datetime.now(timezone.utc).isoformat()}",
        f"git_branch: {os.environ.get('AUTONAV_CALIB_GIT_BRANCH') or git_value(['branch', '--show-current'])}",
        f"git_commit: {os.environ.get('AUTONAV_CALIB_GIT_COMMIT') or git_value(['rev-parse', '--short=12', 'HEAD'])}",
        f"command_line: {argv_text}",
        "",
        "speed_conversions:",
        "  0.25 mph = 0.111760 m/s",
        "  0.50 mph = 0.223520 m/s",
        "  1.00 mph = 0.447040 m/s",
        "  2.00 mph = 0.894080 m/s",
        "  3.00 mph = 1.341120 m/s",
        "",
        "topics:",
    ]
    lines.extend(f"  - {topic}" for topic in topics)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profiles", type=Path, default=Path(__file__).with_name("profiles.yaml"))
    parser.add_argument("--profile")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--validate", action="store_true")
    parser.add_argument("--field", choices=[
        "bag_profile",
        "command_mode",
        "description",
        "duration_s",
        "operator_notes",
        "record_until_interrupt",
        "requires_allow_high_speed",
        "wait_for_auto",
    ])
    parser.add_argument("--write-metadata", type=Path)
    parser.add_argument("--run-name", default="")
    parser.add_argument("--bag-path", default="")
    parser.add_argument("--topic-file", type=Path)
    parser.add_argument("--allow-high-speed", action="store_true")
    parser.add_argument("--raw-lidar", action="store_true")
    parser.add_argument("--argv", nargs=argparse.REMAINDER, default=[])
    args = parser.parse_args()

    config = load_config(args.profiles)

    if args.list:
        for name in sorted(config["profiles"]):
            profile = get_profile(config, name)
            print(f"{name}\t{profile.get('description', '')}")
        return 0

    if args.validate:
        for name in sorted(config["profiles"]):
            validate_profile(name, config)
        print(f"validated {len(config['profiles'])} profiles")
        return 0

    if not args.profile:
        parser.error("--profile is required unless --list or --validate is used")

    validate_profile(args.profile, config)
    profile = get_profile(config, args.profile)

    if args.field:
        value: Any
        if args.field == "duration_s":
            value = duration_seconds(profile)
            print("manual_stop" if value is None else f"{value:.3f}")
        else:
            value = profile.get(args.field, False if args.field.startswith(("requires", "record", "wait")) else "")
            if isinstance(value, bool):
                print("true" if value else "false")
            else:
                print(value)
        return 0

    if args.write_metadata:
        if not args.topic_file:
            parser.error("--topic-file is required with --write-metadata")
        argv_text = " ".join(shlex.quote(part) for part in args.argv)
        write_metadata(
            output=args.write_metadata,
            profiles_path=args.profiles,
            profile_name=args.profile,
            run_name=args.run_name,
            bag_path=args.bag_path,
            topic_file=args.topic_file,
            allow_high_speed=args.allow_high_speed,
            raw_lidar=args.raw_lidar,
            argv_text=argv_text,
        )
        return 0

    parser.error("select --list, --validate, --field, or --write-metadata")
    return 2


if __name__ == "__main__":
    sys.exit(main())
