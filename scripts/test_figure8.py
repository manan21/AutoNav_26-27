#!/usr/bin/env python3
"""Drive the Shogi MuJoCo model around a figure-eight path."""

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


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate2(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def robot_forward_yaw(data: mujoco.MjData, base_body: int) -> float:
    rotation = data.xmat[base_body].reshape(3, 3)
    forward_x = -rotation[0, 1]
    forward_y = -rotation[1, 1]
    return math.atan2(forward_y, forward_x)


def figure8_reference(
    t: float,
    *,
    origin: tuple[float, float],
    path_yaw: float,
    scale: float,
    period: float,
    lookahead: float,
) -> tuple[tuple[float, float], float, float, float]:
    theta = 2.0 * math.pi * ((t + lookahead) % period) / period
    theta_dot = 2.0 * math.pi / period

    local_x = scale * math.sin(theta)
    local_y = 0.5 * scale * math.sin(2.0 * theta)
    local_dx = scale * theta_dot * math.cos(theta)
    local_dy = scale * theta_dot * math.cos(2.0 * theta)
    local_ddx = -scale * theta_dot * theta_dot * math.sin(theta)
    local_ddy = -2.0 * scale * theta_dot * theta_dot * math.sin(2.0 * theta)

    world_x, world_y = rotate2(local_x, local_y, path_yaw)
    tangent_yaw = path_yaw + math.atan2(local_dy, local_dx)
    speed = math.hypot(local_dx, local_dy)
    yaw_rate = 0.0
    if speed > 1e-6:
        yaw_rate = (local_dx * local_ddy - local_dy * local_ddx) / (speed * speed)

    return (origin[0] + world_x, origin[1] + world_y), tangent_yaw, speed, yaw_rate


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


def figure8_wheel_targets(
    data: mujoco.MjData,
    base_body: int,
    args: argparse.Namespace,
    *,
    origin: tuple[float, float],
    path_yaw: float,
) -> tuple[float, float]:
    target, target_yaw, feedforward_speed, feedforward_yaw = figure8_reference(
        data.time,
        origin=origin,
        path_yaw=path_yaw,
        scale=args.path_scale,
        period=args.period,
        lookahead=args.lookahead,
    )

    robot_x = float(data.xpos[base_body, 0])
    robot_y = float(data.xpos[base_body, 1])
    robot_yaw = robot_forward_yaw(data, base_body)

    error_x = target[0] - robot_x
    error_y = target[1] - robot_y
    forward_x = math.cos(robot_yaw)
    forward_y = math.sin(robot_yaw)
    left_x = -math.sin(robot_yaw)
    left_y = math.cos(robot_yaw)
    along_error = error_x * forward_x + error_y * forward_y
    cross_error = error_x * left_x + error_y * left_y
    heading_error = wrap_pi(target_yaw - robot_yaw)

    linear_velocity = feedforward_speed + args.along_gain * along_error
    yaw_velocity = (
        feedforward_yaw
        + args.cross_gain * cross_error
        + args.heading_gain * heading_error
    )

    linear_velocity = clamp(linear_velocity, 0.05, args.linear_speed)
    yaw_velocity = clamp(yaw_velocity, -args.yaw_rate, args.yaw_rate)

    return wheel_targets(
        linear_velocity,
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


def make_path_frame(data: mujoco.MjData, base_body: int) -> tuple[tuple[float, float], float]:
    start = data.xpos[base_body]
    start_yaw = robot_forward_yaw(data, base_body)
    return (float(start[0]), float(start[1])), start_yaw - math.pi / 4.0


def run_headless(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> None:
    left_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_velocity")
    right_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_velocity")
    base_body = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    origin, path_yaw = make_path_frame(data, base_body)
    start = data.xpos[base_body].copy()
    last = start.copy()
    path_length = 0.0
    min_x = max_x = float(start[0])
    min_y = max_y = float(start[1])

    while data.time < args.duration:
        left, right = figure8_wheel_targets(
            data,
            base_body,
            args,
            origin=origin,
            path_yaw=path_yaw,
        )
        set_controls(data, left_actuator, right_actuator, left, right)
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
    print(f"xy bounds: x=[{min_x:.3f}, {max_x:.3f}], y=[{min_y:.3f}, {max_y:.3f}]")


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> None:
    import mujoco.viewer

    left_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_velocity")
    right_actuator = get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_velocity")
    base_body = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    origin, path_yaw = make_path_frame(data, base_body)
    camera_target = data.xpos[base_body].copy()
    camera_target[2] = args.camera_height

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 8.0
        viewer.cam.elevation = -28
        viewer.cam.azimuth = 135
        viewer.cam.lookat[:] = camera_target

        wall_start = time.time()
        while viewer.is_running() and data.time < args.duration:
            step_start = time.time()
            left, right = figure8_wheel_targets(
                data,
                base_body,
                args,
                origin=origin,
                path_yaw=path_yaw,
            )
            set_controls(data, left_actuator, right_actuator, left, right)
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
    parser.add_argument("--duration", type=float, default=40.0, help="Simulation duration in seconds.")
    parser.add_argument("--headless", action="store_true", help="Run without opening the MuJoCo viewer.")
    parser.add_argument("--linear-speed", type=float, default=0.8, help="Maximum forward speed in m/s.")
    parser.add_argument("--yaw-rate", type=float, default=1.4, help="Maximum yaw rate in rad/s.")
    parser.add_argument("--period", type=float, default=20.0, help="Seconds per figure-eight cycle.")
    parser.add_argument("--path-scale", type=float, default=1.5, help="Half-width of the figure-eight path in meters.")
    parser.add_argument("--lookahead", type=float, default=0.35, help="Seconds to look ahead on the path.")
    parser.add_argument("--along-gain", type=float, default=0.35, help="Forward position error gain.")
    parser.add_argument("--cross-gain", type=float, default=1.0, help="Lateral position error gain.")
    parser.add_argument("--heading-gain", type=float, default=1.8, help="Heading error gain.")
    parser.add_argument("--camera-smoothing", type=float, default=0.45, help="Viewer camera follow smoothing time constant in seconds.")
    parser.add_argument("--camera-height", type=float, default=0.45, help="Fixed viewer look-at height above the floor.")
    parser.add_argument("--track-width", type=float, default=0.72, help="Approximate wheel separation in meters.")
    parser.add_argument("--wheel-radius", type=float, default=0.113, help="Approximate wheel radius in meters.")
    parser.add_argument("--left-sign", type=float, default=1.0, choices=(-1.0, 1.0), help="Flip left wheel direction.")
    parser.add_argument("--right-sign", type=float, default=-1.0, choices=(-1.0, 1.0), help="Flip right wheel direction.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    if args.headless:
        run_headless(model, data, args)
    else:
        run_viewer(model, data, args)


if __name__ == "__main__":
    main()
