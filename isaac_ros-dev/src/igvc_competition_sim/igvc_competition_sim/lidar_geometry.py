from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Protocol


class CylinderLike(Protocol):
    center: tuple[float, float]
    radius_m: float
    height_m: float


@dataclass(frozen=True)
class CylinderRayHit:
    x: float
    y: float
    z: float
    range_m: float
    layer: int
    azimuth_rad: float
    elevation_rad: float
    obstacle_name: str


def _ray_circle_near_distance(
    origin_xy: tuple[float, float],
    direction_xy: tuple[float, float],
    center_xy: tuple[float, float],
    radius_m: float,
) -> float | None:
    ox, oy = origin_xy
    dx, dy = direction_xy
    cx, cy = center_xy
    fx = ox - cx
    fy = oy - cy
    half_b = fx * dx + fy * dy
    c = fx * fx + fy * fy - radius_m * radius_m
    discriminant = half_b * half_b - c
    if discriminant < 0.0:
        return None
    root = math.sqrt(discriminant)
    near = -half_b - root
    far = -half_b + root
    if near >= 0.0:
        return near
    if far >= 0.0:
        return far
    return None


def raycast_cylinders(
    cylinders: Iterable[CylinderLike],
    origin_xy: tuple[float, float],
    heading_rad: float,
    sensor_height_m: float,
    azimuth_min_rad: float,
    azimuth_max_rad: float,
    horizontal_resolution_rad: float,
    elevation_min_rad: float,
    elevation_max_rad: float,
    layers: int,
    range_min_m: float,
    range_max_m: float,
) -> list[CylinderRayHit]:
    cylinder_list = tuple(cylinders)
    if not cylinder_list or layers <= 0:
        return []

    azimuth_count = (
        int(math.floor((azimuth_max_rad - azimuth_min_rad)
                       / horizontal_resolution_rad))
        + 1
    )
    hits: list[CylinderRayHit] = []
    for layer in range(layers):
        if layers == 1:
            elevation = 0.5 * (elevation_min_rad + elevation_max_rad)
        else:
            frac = layer / float(layers - 1)
            elevation = elevation_min_rad + (
                elevation_max_rad - elevation_min_rad) * frac
        cos_elevation = math.cos(elevation)
        if cos_elevation <= 1e-6:
            continue
        tan_elevation = math.tan(elevation)

        for azimuth_idx in range(azimuth_count):
            azimuth = azimuth_min_rad + (
                horizontal_resolution_rad * azimuth_idx)
            if azimuth > azimuth_max_rad + 1e-9:
                continue
            world_angle = heading_rad + azimuth
            direction_xy = (math.cos(world_angle), math.sin(world_angle))
            best_hit: CylinderRayHit | None = None
            best_range = math.inf

            for cylinder in cylinder_list:
                horizontal_distance = _ray_circle_near_distance(
                    origin_xy,
                    direction_xy,
                    cylinder.center,
                    float(cylinder.radius_m),
                )
                if horizontal_distance is None:
                    continue
                range_m = horizontal_distance / cos_elevation
                if range_m < range_min_m or range_m > range_max_m:
                    continue
                z = sensor_height_m + horizontal_distance * tan_elevation
                if z < 0.0 or z > float(cylinder.height_m):
                    continue
                if range_m < best_range:
                    best_range = range_m
                    best_hit = CylinderRayHit(
                        x=origin_xy[0] + horizontal_distance
                        * direction_xy[0],
                        y=origin_xy[1] + horizontal_distance
                        * direction_xy[1],
                        z=z,
                        range_m=range_m,
                        layer=layer,
                        azimuth_rad=azimuth,
                        elevation_rad=elevation,
                        obstacle_name=getattr(cylinder, "name", "obstacle"),
                    )
            if best_hit is not None:
                hits.append(best_hit)
    return hits
