#!/usr/bin/env python3
"""Drive the Shogi MuJoCo model in a circle."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import mujoco


DEFAULT_MODEL = (
    Path(__file__).resolve().parents[1]
    / "isaac_ros-dev"
    / "src"
    / "bringup"
    / "description"
    / "shogi.xml"
)


def get_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"Could not find {name!r} in model")
    return obj_id


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smooth_alpha(dt: float, time_constant: float) -> float:
    if time_constant <= 0.0:
        return 1.0
    return 1.0 - math.exp(-dt / time_constant)


def robot_forward_yaw(data: mujoco.MjData, base_body: int) -> float:
    rotation = data.xmat[base_body].reshape(3, 3)
    forward_x = -rotation[0, 1]
    forward_y = -rotation[1, 1]
    return math.atan2(forward_y, forward_x)


def wheel_targets(
    linear_velocity: float,
    yaw_velocity: float,
    *,
    track_width: float,
    wheel_radius: float,
    left_sign: float,
    right_sign: float,
) -> tuple[float, float]:
    left_linear = linear_velocity - 0.5 * track_width * yaw_velocity
    right_linear = linear_velocity + 0.5 * track_width * yaw_velocity
    left = left_sign * left_linear / wheel_radius
    right = right_sign * right_linear / wheel_radius
    return left, right


def slew_target(current: float, target: float, rate_limit: float, dt: float) -> float:
    step = rate_limit * dt
    return current + clamp(target - current, -step, step)


def circle_wheel_targets(args: argparse.Namespace) -> tuple[float, float]:
    physical_turn_sign = -1.0 if args.clockwise else 1.0
    yaw_velocity = -physical_turn_sign * args.linear_speed / args.circle_radius
    yaw_velocity = clamp(yaw_velocity, -args.yaw_rate, args.yaw_rate)

    return wheel_targets(
        args.linear_speed,
        yaw_velocity,
        track_width=args.track_width,
        wheel_radius=args.wheel_radius,
        left_sign=args.left_sign,
        right_sign=args.right_sign,
    )


def set_controls(
    data: mujoco.MjData,
    left_actuator: int,
    right_actuator: int,
    left_target: float,
    right_target: float,
) -> None:
    data.ctrl[left_actuator] = left_target
    data.ctrl[right_actuator] = right_target


def run_headless(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> None:
    left_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_velocity")
    right_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_velocity")
    base_body = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    start = data.xpos[base_body].copy()
    start_yaw = robot_forward_yaw(data, base_body)
    physical_turn_sign = -1.0 if args.clockwise else 1.0
    center_x = float(start[0]) - physical_turn_sign * args.circle_radius * math.sin(start_yaw)
    center_y = float(start[1]) + physical_turn_sign * args.circle_radius * math.cos(start_yaw)
    last = start.copy()
    path_length = 0.0
    min_x = max_x = float(start[0])
    min_y = max_y = float(start[1])
    left_cmd = 0.0
    right_cmd = 0.0

    while data.time < args.duration:
        left_target, right_target = circle_wheel_targets(args)
        left_cmd = slew_target(left_cmd, left_target, args.wheel_accel_limit, model.opt.timestep)
        right_cmd = slew_target(right_cmd, right_target, args.wheel_accel_limit, model.opt.timestep)
        set_controls(data, left_actuator, right_actuator, left_cmd, right_cmd)
        mujoco.mj_step(model, data)

        pos = data.xpos[base_body]
        path_length += math.hypot(float(pos[0] - last[0]), float(pos[1] - last[1]))
        last = pos.copy()
        min_x = min(min_x, float(pos[0]))
        max_x = max(max_x, float(pos[0]))
        min_y = min(min_y, float(pos[1]))
        max_y = max(max_y, float(pos[1]))

    end = data.xpos[base_body].copy()
    print(f"simulated {data.time:.2f}s")
    print(f"start xyz: {start[0]:.3f} {start[1]:.3f} {start[2]:.3f}")
    print(f"end xyz:   {end[0]:.3f} {end[1]:.3f} {end[2]:.3f}")
    print(f"path length xy: {path_length:.3f} m")
    print(f"commanded circle center xy: {center_x:.3f} {center_y:.3f}")
    print(f"xy bounds: x=[{min_x:.3f}, {max_x:.3f}], y=[{min_y:.3f}, {max_y:.3f}]")


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> None:
    import mujoco.viewer

    left_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_velocity")
    right_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_velocity")
    base_body = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    camera_target = data.xpos[base_body].copy()
    camera_target[2] = args.camera_height

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.5 * args.circle_radius + 3.0
        viewer.cam.elevation = -28
        viewer.cam.azimuth = 135
        viewer.cam.lookat[:] = camera_target

        wall_start = time.time()
        left_cmd = 0.0
        right_cmd = 0.0
        while viewer.is_running() and data.time < args.duration:
            step_start = time.time()
            left_target, right_target = circle_wheel_targets(args)
            left_cmd = slew_target(left_cmd, left_target, args.wheel_accel_limit, model.opt.timestep)
            right_cmd = slew_target(right_cmd, right_target, args.wheel_accel_limit, model.opt.timestep)
            set_controls(data, left_actuator, right_actuator, left_cmd, right_cmd)
            mujoco.mj_step(model, data)

            desired_target = data.xpos[base_body].copy()
            desired_target[2] = args.camera_height
            alpha = smooth_alpha(model.opt.timestep, args.camera_smoothing)
            camera_target += alpha * (desired_target - camera_target)
            viewer.cam.lookat[:] = camera_target
            viewer.sync()

            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print(f"viewer ran for {time.time() - wall_start:.2f}s wall time, {data.time:.2f}s sim time")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="Path to the MuJoCo XML model.")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration in seconds.")
    parser.add_argument("--headless", action="store_true", help="Run without opening the MuJoCo viewer.")
    parser.add_argument("--linear-speed", type=float, default=0.45, help="Forward speed in m/s.")
    parser.add_argument("--circle-radius", type=float, default=2.0, help="Commanded circle radius in meters.")
    parser.add_argument("--yaw-rate", type=float, default=0.45, help="Maximum yaw rate in rad/s.")
    parser.add_argument("--clockwise", action="store_true", help="Drive clockwise instead of counter-clockwise.")
    parser.add_argument("--wheel-accel-limit", type=float, default=18.0, help="Maximum wheel velocity command change in rad/s^2.")
    parser.add_argument("--camera-smoothing", type=float, default=0.45, help="Viewer camera follow smoothing time constant in seconds.")
    parser.add_argument("--camera-height", type=float, default=0.45, help="Fixed viewer look-at height above the floor.")
    parser.add_argument("--track-width", type=float, default=0.72, help="Approximate wheel separation in meters.")
    parser.add_argument("--wheel-radius", type=float, default=0.113, help="Approximate wheel radius in meters.")
    parser.add_argument("--left-sign", type=float, default=1.0, choices=(-1.0, 1.0), help="Flip left wheel direction.")
    parser.add_argument("--right-sign", type=float, default=-1.0, choices=(-1.0, 1.0), help="Flip right wheel direction.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.circle_radius <= 0.0:
        raise ValueError("--circle-radius must be greater than zero")
    if args.linear_speed < 0.0:
        raise ValueError("--linear-speed must be non-negative")

    model_path = args.model.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    if args.headless:
        run_headless(model, data, args)
    else:
        run_viewer(model, data, args)


if __name__ == "__main__":
    main()
