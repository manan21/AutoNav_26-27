#!/usr/bin/env python3
"""Offline course feasibility validator (FROZEN harness component).

Without running the sim, confirm each authored course is at least geometrically
feasible for the padded robot: build a costmap-resolution occupancy grid where
tapes / obstacles / potholes are lethal, inflate by the robot inscribed radius
(physical_half_width + padding), and verify that start -> every mission waypoint
-> finish lie in one connected free region (8-connected). Catches gross errors
(a gap narrower than the robot, a waypoint inside an obstacle/tape) that would
otherwise only surface as a wasted sim run.

NOTE: this is a necessary (not sufficient) feasibility check -- it uses a
circular inscribed-radius inflation, so it can pass a course the kinodynamic
planner still finds hard. Route feasibility is confirmed for real on the first
sim baseline. ASCII output only. Exit 0 if all requested courses pass.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation, label

# import the FROZEN course loader from the package
PKG = Path(__file__).resolve().parents[2]  # .../igvc_competition_sim
sys.path.insert(0, str(PKG))
from igvc_competition_sim.course import load_course, course_bounds  # noqa: E402

RES = 0.05


def _disk(radius_cells: int) -> np.ndarray:
    r = max(1, int(radius_cells))
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def validate(course_path: Path) -> tuple[bool, str]:
    c = load_course(course_path)
    min_x, min_y, max_x, max_y = course_bounds(c, margin_m=2.0)
    nx = int(math.ceil((max_x - min_x) / RES))
    ny = int(math.ceil((max_y - min_y) / RES))
    lethal = np.zeros((ny, nx), dtype=bool)

    def w2c(x: float, y: float) -> tuple[int, int]:
        return (int((x - min_x) / RES), int((y - min_y) / RES))

    def stamp(x: float, y: float, radius_m: float) -> None:
        cx, cy = w2c(x, y)
        r = max(1, int(round(radius_m / RES)))
        x0, x1 = max(0, cx - r), min(nx, cx + r + 1)
        y0, y1 = max(0, cy - r), min(ny, cy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        ys, xs = np.ogrid[y0:y1, x0:x1]
        lethal[y0:y1, x0:x1] |= ((xs - cx) ** 2 + (ys - cy) ** 2) <= r * r

    # tapes (sample along each segment)
    for t in c.tapes:
        (ax, ay), (bx, by) = t.start, t.end
        length = math.hypot(bx - ax, by - ay)
        n = max(1, int(length / (RES * 0.5)))
        for i in range(n + 1):
            s = i / n
            stamp(ax + (bx - ax) * s, ay + (by - ay) * s, t.width_m * 0.5)
    for o in c.obstacles:
        stamp(o.center[0], o.center[1], o.radius_m)
    for p in c.potholes:
        stamp(p.center[0], p.center[1], p.radius_m)

    # inflate by inscribed radius (half-width + padding); the robot cannot be
    # closer than this to any lethal cell without a footprint violation.
    r_in = c.robot.physical_half_width_m + c.robot.footprint_padding_m
    inflated = binary_dilation(lethal, structure=_disk(round(r_in / RES)))
    free = ~inflated

    labels, _ = label(free)

    def cell_ok(x: float, y: float, name: str) -> tuple[bool, int, str]:
        cx, cy = w2c(x, y)
        if not (0 <= cx < nx and 0 <= cy < ny):
            return False, -1, f"{name} out of bounds"
        if inflated[cy, cx]:
            return False, -1, f"{name} ({x:.2f},{y:.2f}) inside lethal/inflated"
        return True, labels[cy, cx], ""

    ok_s, start_lab, msg_s = cell_ok(c.start.x, c.start.y, "start")
    if not ok_s:
        return False, msg_s
    problems = []
    targets = [(wp.label, wp.x_m, wp.y_m) for wp in c.mission_waypoints]
    targets.append(("finish", c.finish[0], c.finish[1]))
    for name, x, y in targets:
        ok, lab, msg = cell_ok(x, y, name)
        if not ok:
            problems.append(msg)
        elif lab != start_lab:
            problems.append(f"{name} ({x:.2f},{y:.2f}) not connected to start")
    free_frac = float(free.mean())
    detail = (f"grid={nx}x{ny} r_in={r_in:.2f}m free={free_frac*100:.0f}% "
              f"wp={len(c.mission_waypoints)}")
    if problems:
        return False, detail + " | " + "; ".join(problems)
    return True, detail + " | all waypoints + finish reachable from start"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("courses", nargs="+")
    args = ap.parse_args()
    all_ok = True
    print("=== COURSE FEASIBILITY VALIDATION (padded-robot connectivity) ===")
    for cp in args.courses:
        try:
            ok, msg = validate(Path(cp))
        except Exception as exc:  # noqa: BLE001
            ok, msg = False, f"EXCEPTION: {exc}"
        all_ok &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {Path(cp).stem}: {msg}")
    print("RESULT: " + ("ALL PASS" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
