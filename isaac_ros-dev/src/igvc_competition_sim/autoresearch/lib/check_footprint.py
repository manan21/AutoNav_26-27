#!/usr/bin/env python3
"""C-i footprint accuracy verification (FROZEN harness component).

Cross-checks the robot footprint across the four places it must agree, so the
generated global path accounts for the *true* physical robot and the loop can
never silently shrink it:

  1. Course YAML `robot:` block  -> RobotSpec used by the FROZEN scorer
     (course_monitor.py builds the padded violation box from these).
  2. nav2_params_camera.yaml     -> footprint polygon + padding in BOTH
     local_costmap and global_costmap (what the planner/controller use).
  3. shogi.urdf nav_center_joint -> base_link -> nav_center offset.
  4. bt_nav.xml PathFootprintSafe -> the runtime global footprint gate.

Exit 0 = all consistent to TOL. Exit 1 = a mismatch (fail loud). No ROS needed.
ASCII output only.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

TOL = 1e-6

REPO = Path(__file__).resolve().parents[5]  # .../AutoNav_25-26
SRC = REPO / "isaac_ros-dev" / "src"
DEF_COURSE = SRC / "igvc_competition_sim" / "config" / "igvc_competition_compact.yaml"
DEF_NAV2 = SRC / "slam" / "config" / "nav2_params_camera.yaml"
DEF_URDF = SRC / "bringup" / "description" / "shogi.urdf"
DEF_BT = SRC / "slam" / "behavior_trees" / "bt_nav.xml"


def _fail(msg: str) -> None:
    print("  FAIL: " + msg)


def parse_robot_block(course_yaml: Path) -> dict:
    """Read physical dims from the course YAML robot: block (no yaml dep needed
    for these flat float keys, but use yaml if present)."""
    text = course_yaml.read_text(encoding="utf-8")
    keys = (
        "base_link_to_nav_center_m",
        "physical_half_length_m",
        "physical_half_width_m",
        "footprint_padding_m",
    )
    out: dict = {}
    for k in keys:
        m = re.search(rf"^\s*{k}\s*:\s*([-\d.eE+]+)", text, re.MULTILINE)
        if not m:
            raise ValueError(f"{k} not found in {course_yaml}")
        out[k] = float(m.group(1))
    return out


def parse_footprint_polygon(s: str) -> tuple[float, float]:
    """Return (half_length_x, half_width_y) from a '[[x,y],...]' string."""
    nums = [float(v) for v in re.findall(r"[-\d.eE+]+", s)]
    if len(nums) < 8 or len(nums) % 2 != 0:
        raise ValueError(f"unexpected footprint polygon: {s!r}")
    xs = [abs(nums[i]) for i in range(0, len(nums), 2)]
    ys = [abs(nums[i]) for i in range(1, len(nums), 2)]
    return max(xs), max(ys)


def parse_nav2(nav2_yaml: str) -> list[dict]:
    text = Path(nav2_yaml).read_text(encoding="utf-8")
    results = []
    for scope in ("local_costmap", "global_costmap"):
        # search the file for the footprint/padding/base_frame after scope label
        idx = text.find(scope + ":")
        if idx < 0:
            continue
        sub = text[idx:]
        fp = re.search(r'footprint:\s*"(.*?)"', sub)
        pad = re.search(r"footprint_padding:\s*([-\d.eE+]+)", sub)
        base = re.search(r"robot_base_frame:\s*(\S+)", sub)
        if fp and pad:
            hl, hw = parse_footprint_polygon(fp.group(1))
            results.append({
                "scope": scope,
                "half_length": hl,
                "half_width": hw,
                "padding": float(pad.group(1)),
                "base_frame": base.group(1) if base else "?",
            })
    return results


def parse_urdf_nav_center(urdf: Path) -> float:
    text = urdf.read_text(encoding="utf-8")
    m = re.search(r'name="nav_center_joint".*?<origin\s+xyz="([-\d.eE+]+)',
                  text, re.DOTALL)
    if not m:
        raise ValueError("nav_center_joint origin not found in URDF")
    return float(m.group(1))


def parse_bt_footprint(bt: Path) -> dict | None:
    text = bt.read_text(encoding="utf-8")
    m = re.search(r"<PathFootprintSafe\b(.*?)/>", text, re.DOTALL)
    if not m:
        return None
    blk = m.group(1)
    fp = re.search(r'footprint="(.*?)"', blk)
    pad = re.search(r'footprint_padding="([-\d.eE+]+)"', blk)
    base = re.search(r'robot_base_frame="(\S+?)"', blk)
    if not fp:
        return None
    hl, hw = parse_footprint_polygon(fp.group(1))
    return {
        "half_length": hl,
        "half_width": hw,
        "padding": float(pad.group(1)) if pad else None,
        "base_frame": base.group(1) if base else "?",
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--course", default=str(DEF_COURSE))
    ap.add_argument("--nav2", default=str(DEF_NAV2))
    ap.add_argument("--urdf", default=str(DEF_URDF))
    ap.add_argument("--bt", default=str(DEF_BT))
    args = ap.parse_args()

    print("=== C-i FOOTPRINT ACCURACY CHECK ===")
    ok = True

    rb = parse_robot_block(Path(args.course))
    canon_hl = rb["physical_half_length_m"]
    canon_hw = rb["physical_half_width_m"]
    canon_pad = rb["footprint_padding_m"]
    canon_nav = rb["base_link_to_nav_center_m"]
    print(f"canon (course RobotSpec): half_length={canon_hl} half_width={canon_hw} "
          f"padding={canon_pad} nav_center={canon_nav}")

    nav2 = parse_nav2(args.nav2)
    if not nav2:
        ok = False
        _fail("could not parse any costmap footprint from nav2 params")
    for c in nav2:
        if abs(c["half_length"] - canon_hl) > TOL:
            ok = False
            _fail(f"{c['scope']} half_length {c['half_length']} != {canon_hl}")
        if abs(c["half_width"] - canon_hw) > TOL:
            ok = False
            _fail(f"{c['scope']} half_width {c['half_width']} != {canon_hw}")
        if abs(c["padding"] - canon_pad) > TOL:
            ok = False
            _fail(f"{c['scope']} padding {c['padding']} != {canon_pad}")
        if c["base_frame"] != "nav_center":
            ok = False
            _fail(f"{c['scope']} robot_base_frame {c['base_frame']} != nav_center")
        print(f"  nav2 {c['scope']}: half_length={c['half_length']} "
              f"half_width={c['half_width']} padding={c['padding']} "
              f"base_frame={c['base_frame']}  OK")

    nav_urdf = parse_urdf_nav_center(Path(args.urdf))
    if abs(nav_urdf - canon_nav) > TOL:
        ok = False
        _fail(f"URDF nav_center offset {nav_urdf} != {canon_nav}")
    else:
        print(f"  URDF nav_center_joint x={nav_urdf}  OK")

    bt = parse_bt_footprint(Path(args.bt))
    if bt is None:
        print("  bt_nav.xml: PathFootprintSafe footprint not found (WARN, not fatal)")
    else:
        if abs(bt["half_length"] - canon_hl) > TOL:
            ok = False
            _fail(f"BT half_length {bt['half_length']} != {canon_hl}")
        if abs(bt["half_width"] - canon_hw) > TOL:
            ok = False
            _fail(f"BT half_width {bt['half_width']} != {canon_hw}")
        if bt["padding"] is not None and abs(bt["padding"] - canon_pad) > TOL:
            ok = False
            _fail(f"BT padding {bt['padding']} != {canon_pad}")
        print(f"  bt PathFootprintSafe: half_length={bt['half_length']} "
              f"half_width={bt['half_width']} padding={bt['padding']} "
              f"base_frame={bt['base_frame']}  OK")

    pad_hl = canon_hl + canon_pad
    pad_hw = canon_hw + canon_pad
    print(f"padded footprint (scorer box) = +/-{pad_hl:.3f} x +/-{pad_hw:.3f} m "
          f"at nav_center (+{canon_nav} fwd of base_link)")
    print("RESULT: " + ("PASS - footprint consistent across all sources"
                        if ok else "FAIL - footprint mismatch (see above)"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
