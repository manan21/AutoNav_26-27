#!/usr/bin/env python3
"""Per-run metric extraction for the auto_camera build-loop (FROZEN harness).

Reads one RUN_DIR ( {bag/, final_score.txt, mission.log} ) and returns a metrics
dict driving the speed-weighted/reliability-gated fitness. Self-contained: parses
the rosbag2 directly via rosbag2_py (topic types read from bag metadata, not
hardcoded) so it can be unit-tested on a synthetic bag without a live sim.

Authoritative reliability verdict = the FROZEN course_monitor's score JSON
(final_score.txt, else last /igvc_sim/score in bag). Everything else is detail.

Every metric is computed in isolation; on any failure it degrades to None (NA)
and the run is flagged INCOMPLETE rather than crashing. ASCII only.

CLI: python3 metrics.py RUN_DIR [--course COURSE_YAML]  -> prints JSON dict.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any

# --- action_msgs/GoalStatus codes ---
ST_ACCEPTED = 1
ST_EXECUTING = 2
ST_SUCCEEDED = 4

# recovery log markers (exact strings from the plugin sources)
MARK_BREADCRUMB = "BreadcrumbReverse: reversing to breadcrumb"
MARK_GRADIENT = "GradientEscape: starting"
MARK_PFS_REJECT = "PathFootprintSafe: rejecting path"
MARK_CLEAR = "Clearing"  # nav2 ClearCostmap service logs vary; best-effort

STUCK_GAP_S = 1.0          # nonzero-cmd gap longer than this while not at goal
SLOW_SPEED = 0.05          # |linear.x| below this counts as "not moving"


def _read_bag(bag_dir: Path) -> tuple[dict[str, list[tuple[int, Any]]], dict[str, str]]:
    """Return {topic: [(t_ns, msg), ...]} and {topic: type_str}. Empty on failure."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    # rosbag2 RUN_DIR/bag is itself a folder with metadata.yaml + .db3/.mcap
    uri = str(bag_dir)
    storage_id = ""
    # detect storage by file extension
    has_mcap = any(p.suffix == ".mcap" for p in Path(bag_dir).glob("*.mcap"))
    storage_id = "mcap" if has_mcap else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes: dict[str, Any] = {}
    out: dict[str, list[tuple[int, Any]]] = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        ty = types.get(topic)
        if ty is None:
            continue
        cls = msg_classes.get(ty)
        if cls is None:
            try:
                cls = get_message(ty)
            except Exception:
                cls = False
            msg_classes[ty] = cls
        if cls is False:
            continue
        try:
            msg = deserialize_message(data, cls)
        except Exception:
            continue
        out.setdefault(topic, []).append((t_ns, msg))
    return out, types


def _clock_mapper(msgs: dict[str, list[tuple[int, Any]]]):
    """Build wall_ns -> sim_seconds interpolator from /clock; identity fallback."""
    clk = msgs.get("/clock") or []
    pairs = []
    for t_ns, m in clk:
        try:
            sim = float(m.clock.sec) + float(m.clock.nanosec) * 1e-9
            pairs.append((t_ns, sim))
        except Exception:
            continue
    if len(pairs) < 2:
        return lambda t_ns: t_ns * 1e-9  # fall back to wall seconds

    pairs.sort()
    ts = [p[0] for p in pairs]
    sims = [p[1] for p in pairs]

    def to_sim(t_ns: int) -> float:
        if t_ns <= ts[0]:
            return sims[0]
        if t_ns >= ts[-1]:
            return sims[-1]
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t_ns:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = 0.0 if span == 0 else (t_ns - ts[lo]) / span
        return sims[lo] + f * (sims[hi] - sims[lo])

    return to_sim


def _load_score(run_dir: Path, msgs: dict) -> dict | None:
    fp = run_dir / "final_score.txt"
    if fp.is_file():
        txt = fp.read_text(encoding="utf-8", errors="replace")
        # `ros2 topic echo --once` prints YAML-ish 'data: "{...json...}"'
        # try to extract the embedded JSON object
        start = txt.find("{")
        end = txt.rfind("}")
        if start >= 0 and end > start:
            blob = txt[start:end + 1].replace("\\\"", "\"").replace("''", "")
            try:
                return json.loads(blob)
            except Exception:
                pass
    # fall back to last /igvc_sim/score String in the bag
    score_msgs = msgs.get("/igvc_sim/score") or []
    if score_msgs:
        try:
            return json.loads(score_msgs[-1][1].data)
        except Exception:
            return None
    return None


def _mission_completed(run_dir: Path) -> bool | None:
    """Parse mission.log per-waypoint lines: completed iff every waypoint
    succeeded=True and the runner logged 'mission complete'."""
    fp = run_dir / "mission.log"
    if not fp.is_file():
        return None
    txt = fp.read_text(encoding="utf-8", errors="replace")
    if "mission complete" in txt:
        return True
    if "succeeded=False" in txt or "mission aborted" in txt:
        return False
    return None


def _count_action_starts(status_msgs: list[tuple[int, Any]]) -> int:
    """Count distinct goals that reached ACCEPTED/EXECUTING (=activations)."""
    seen = set()
    for _t, m in status_msgs:
        for st in getattr(m, "status_list", []):
            try:
                if st.status in (ST_ACCEPTED, ST_EXECUTING):
                    gid = bytes(st.goal_info.goal_id.uuid)
                    seen.add(gid)
            except Exception:
                continue
    return len(seen)


def _traversal_time(nav_status: list[tuple[int, Any]], to_sim) -> float | None:
    first_exec = None
    last_succ = None
    for t_ns, m in nav_status:
        for st in getattr(m, "status_list", []):
            s = getattr(st, "status", 0)
            if s == ST_EXECUTING and first_exec is None:
                first_exec = to_sim(t_ns)
            if s == ST_SUCCEEDED:
                last_succ = to_sim(t_ns)
    if first_exec is None or last_succ is None or last_succ < first_exec:
        return None
    return round(last_succ - first_exec, 2)


def _count_rosout(rosout: list[tuple[int, Any]], marker: str) -> int:
    n = 0
    for _t, m in rosout:
        try:
            if marker in str(m.msg):
                n += 1
        except Exception:
            continue
    return n


def _cmd_jitter(cmd: list[tuple[int, Any]], to_sim):
    """angular sign reversals, angular variance, stuck events, time_below_speed."""
    if not cmd:
        return None, None, None, None
    angs, lins, times = [], [], []
    for t_ns, m in cmd:
        try:
            angs.append(float(m.angular.z))
            lins.append(float(m.linear.x))
            times.append(to_sim(t_ns))
        except Exception:
            continue
    if len(angs) < 2:
        return 0, 0.0, 0, 0.0
    reversals = 0
    for i in range(1, len(angs)):
        a, b = angs[i - 1], angs[i]
        if a * b < 0 and (abs(a) > 0.02 or abs(b) > 0.02):
            reversals += 1
    ang_var = round(statistics.pvariance(angs), 5)
    # stuck: gaps in nonzero linear cmd; time_below_speed: cumulative slow time
    stuck = 0
    time_below = 0.0
    last_move_t = times[0]
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt < 0:
            continue
        if abs(lins[i]) < SLOW_SPEED:
            time_below += dt
            if times[i] - last_move_t > STUCK_GAP_S:
                stuck += 1
                last_move_t = times[i]  # avoid double-counting the same gap
        else:
            last_move_t = times[i]
    return reversals, ang_var, stuck, round(time_below, 2)


def _first_stamp(msgs: list[tuple[int, Any]], to_sim, origin: float | None) -> float | None:
    if not msgs or origin is None:
        return None
    return round(to_sim(msgs[0][0]) - origin, 2)


def _min_course_clearance(course_yaml: str | None, odom: list[tuple[int, Any]]) -> float | None:
    """Min signed clearance of the padded footprint to nearest tape/obstacle over
    the executed trajectory, using the FROZEN course geometry (mirrors
    course_monitor math). Positive = clear. Reliable, no costmap needed."""
    if not course_yaml or not odom:
        return None
    try:
        import sys as _sys
        pkg = Path(__file__).resolve().parents[2]
        if str(pkg) not in _sys.path:
            _sys.path.insert(0, str(pkg))
        from igvc_competition_sim.course import load_course
        c = load_course(course_yaml)
    except Exception:
        return None
    r = c.robot
    hx = r.physical_half_length_m + r.footprint_padding_m
    hy = r.physical_half_width_m + r.footprint_padding_m
    nav_off = r.base_link_to_nav_center_m

    def yaw_of(m):
        q = m.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def w2nav(x, y, nx, ny, yaw):
        dx, dy = x - nx, y - ny
        cs, sn = math.cos(yaw), math.sin(yaw)
        return cs * dx + sn * dy, -sn * dx + cs * dy

    def clear_to_segment(start, end, half_w, nx, ny, yaw):
        # sample segment, return min over samples of box-edge clearance
        ax, ay = start
        bx, by = end
        length = math.hypot(bx - ax, by - ay)
        n = max(1, int(length / 0.1))
        best = float("inf")
        for i in range(n + 1):
            s = i / n
            lx, ly = w2nav(ax + (bx - ax) * s, ay + (by - ay) * s, nx, ny, yaw)
            dx = abs(lx) - hx
            dy = abs(ly) - hy
            # signed distance from padded box to the tape centerline, minus half width
            if dx <= 0 and dy <= 0:
                d = max(dx, dy)  # inside box (negative)
            else:
                d = math.hypot(max(dx, 0.0), max(dy, 0.0))
            best = min(best, d - half_w)
        return best

    min_clear = float("inf")
    step = max(1, len(odom) // 600)  # cap work
    for idx in range(0, len(odom), step):
        m = odom[idx][1]
        try:
            bx = float(m.pose.pose.position.x)
            by = float(m.pose.pose.position.y)
            yaw = yaw_of(m)
        except Exception:
            continue
        nx = bx + nav_off * math.cos(yaw)
        ny = by + nav_off * math.sin(yaw)
        for t in c.tapes:
            min_clear = min(min_clear, clear_to_segment(
                t.start, t.end, t.width_m * 0.5, nx, ny, yaw))
        for o in c.obstacles:
            lx, ly = w2nav(o.center[0], o.center[1], nx, ny, yaw)
            dx, dy = abs(lx) - hx, abs(ly) - hy
            d = (max(dx, dy) if (dx <= 0 and dy <= 0)
                 else math.hypot(max(dx, 0.0), max(dy, 0.0)))
            min_clear = min(min_clear, d - o.radius_m)
    return round(min_clear, 3) if min_clear != float("inf") else None


def compute_metrics(run_dir: str | Path, course_yaml: str | None = None) -> dict:
    run_dir = Path(run_dir)
    bag_dir = run_dir / "bag"
    m: dict[str, Any] = {"run_dir": str(run_dir)}
    msgs: dict[str, list] = {}
    try:
        if bag_dir.is_dir():
            msgs, _types = _read_bag(bag_dir)
    except Exception as exc:  # noqa: BLE001
        m["bag_error"] = str(exc)
    to_sim = _clock_mapper(msgs)

    # --- authoritative reliability ---
    score = _load_score(run_dir, msgs)
    if score is None:
        m["score_loaded"] = False
        m["failed"] = None
        m["finish_reached"] = None
        m["violations"] = None
    else:
        m["score_loaded"] = True
        m["failed"] = bool(score.get("failed"))
        m["finish_reached"] = bool(score.get("finish_reached"))
        m["violations"] = list(score.get("failures", []))
        m["distance_m"] = score.get("distance_m")
        m["max_speed_mps"] = score.get("max_speed_mps")
    m["mission_completed"] = _mission_completed(run_dir)

    # --- traversal time + first-motion origin ---
    nav_status = msgs.get("/navigate_to_pose/_action/status") or []
    m["traversal_time"] = _traversal_time(nav_status, to_sim)
    origin = None
    for t_ns, msg in nav_status:
        if any(getattr(st, "status", 0) == ST_EXECUTING
               for st in getattr(msg, "status_list", [])):
            origin = to_sim(t_ns)
            break

    # --- recovery activations ---
    rosout = msgs.get("/rosout") or []
    m["breadcrumb"] = _count_rosout(rosout, MARK_BREADCRUMB)
    m["gradient"] = _count_rosout(rosout, MARK_GRADIENT)
    m["pathfootprint_rejects"] = _count_rosout(rosout, MARK_PFS_REJECT)
    m["backup"] = _count_action_starts(msgs.get("/back_up/_action/status") or [])
    m["spin"] = _count_action_starts(msgs.get("/spin/_action/status") or [])
    # clearcostmap: best-effort from rosout
    m["clearcostmap"] = _count_rosout(rosout, "ClearCostmap") + \
        _count_rosout(rosout, "clearing around")

    # --- jitter / stuck ---
    rev, var, stuck, below = _cmd_jitter(msgs.get("/cmd_vel") or [], to_sim)
    m["ang_reversals"] = rev
    m["ang_var"] = var
    m["stuck_events"] = stuck
    m["time_below_speed"] = below

    # --- perception health ---
    m["line_first_s"] = _first_stamp(msgs.get("/line_points") or [], to_sim, origin)
    m["pca_first_s"] = _first_stamp(msgs.get("/scan_pca_filtered") or [], to_sim, origin)

    # --- clearance to course geometry (reliable, geometry-based) ---
    odom = msgs.get("/odom") or msgs.get("/local_ekf/odom") or []
    m["min_course_clear"] = _min_course_clearance(course_yaml, odom)

    # --- costmap-based diagnostics (best-effort; NA if not computed here) ---
    # executed_lethal_clear / plan_inscribed_clear / global_clear_events are
    # produced by the FROZEN scripts/analyze_*.py against the recorded costmaps;
    # left NA here and filled by evaluate.py when those analyzers are wired in.
    m.setdefault("executed_lethal_clear", None)
    m.setdefault("plan_inscribed_clear", None)
    m.setdefault("global_clear_events", None)
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--course", default=None)
    args = ap.parse_args()
    print(json.dumps(compute_metrics(args.run_dir, args.course), indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
