#!/usr/bin/env python3
"""Self-test for metrics.py + fitness.py using a synthetic rosbag2 (no sim).

Writes a small bag with standard message types exercising every metric path,
runs compute_metrics() + fitness.evaluate_candidate(), and asserts the results.
Run in a sourced ROS 2 Humble env: python3 selftest_metrics.py
Exit 0 = all asserts pass.
"""
from __future__ import annotations

import json
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import metrics as M          # noqa: E402
import fitness as F          # noqa: E402

import rosbag2_py            # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from action_msgs.msg import GoalStatusArray, GoalStatus, GoalInfo  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402

COURSE = str(HERE.parent / "courses" / "compact_baseline.yaml")


def wall(sim_s: float) -> int:
    return int((sim_s + 1.0) * 1e9)   # wall = sim + 1.0 s (recoverable by mapper)


def time_msg(sim_s: float) -> Time:
    return Time(sec=int(sim_s), nanosec=int((sim_s % 1) * 1e9))


def make_status(status: int, sim_s: float, gid: int) -> GoalStatusArray:
    arr = GoalStatusArray()
    gs = GoalStatus()
    gi = GoalInfo()
    u = UUID()
    u.uuid = [gid % 256] * 16
    gi.goal_id = u
    gi.stamp = time_msg(sim_s)
    gs.goal_info = gi
    gs.status = status
    arr.status_list = [gs]
    return arr


def write_bag(run_dir: Path, *, fail: bool = False) -> None:
    bag = run_dir / "bag"
    if bag.exists():
        shutil.rmtree(bag)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/navigate_to_pose/_action/status": "action_msgs/msg/GoalStatusArray",
        "/cmd_vel": "geometry_msgs/msg/Twist",
        "/rosout": "rcl_interfaces/msg/Log",
        "/odom": "nav_msgs/msg/Odometry",
        "/igvc_sim/score": "std_msgs/msg/String",
        "/scan_pca_filtered": "sensor_msgs/msg/LaserScan",
        "/back_up/_action/status": "action_msgs/msg/GoalStatusArray",
    }
    for name, ty in topics.items():
        w.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=ty, serialization_format="cdr"))

    def put(topic, msg, sim_s):
        w.write(topic, serialize_message(msg), wall(sim_s))

    # clock every 0.5 s over 0..20 s
    s = 0.0
    while s <= 20.0:
        c = Clock()
        c.clock = time_msg(s)
        put("/clock", c, s)
        s += 0.5

    # nav action: EXECUTING @2s, SUCCEEDED @18s
    put("/navigate_to_pose/_action/status", make_status(2, 2.0, 1), 2.0)
    put("/navigate_to_pose/_action/status", make_status(4, 18.0, 1), 18.0)

    # one backup activation
    put("/back_up/_action/status", make_status(1, 9.0, 7), 9.0)

    # cmd_vel: forward with angular sign reversals, plus a slow gap 10..12s
    t = 2.0
    ang = 0.3
    while t <= 18.0:
        tw = Twist()
        if 10.0 <= t < 12.0:
            tw.linear.x = 0.0     # stuck gap (>1s) while not at goal
            tw.angular.z = 0.0
        else:
            tw.linear.x = 0.25
            ang = -ang            # sign flip every sample
            tw.angular.z = ang
        put("/cmd_vel", tw, t)
        t += 0.25

    # rosout: one each breadcrumb-reverse, gradient-start, PFS reject
    for sim_s, txt in [
        (5.0, "BreadcrumbReverse: reversing to breadcrumb (1.00, 2.00) in frame 'odom'"),
        (6.0, "GradientEscape: starting (threshold=200, speed=0.10, timeout=15.0s)"),
        (7.0, "PathFootprintSafe: rejecting path at pose 3/40 (1.20, 0.30); footprint overlaps"),
    ]:
        lg = Log()
        lg.stamp = time_msg(sim_s)
        lg.msg = txt
        put("/rosout", lg, sim_s)

    # odom: drive forward along lane center from (0,0); stays clear of tape
    s = 2.0
    x = 0.0
    while s <= 18.0:
        od = Odometry()
        od.pose.pose.position.x = x
        od.pose.pose.position.y = 0.0
        od.pose.pose.orientation.w = 1.0
        put("/odom", od, s)
        x += 0.1
        s += 0.5

    # pca scan present at 3s
    put("/scan_pca_filtered", LaserScan(), 3.0)

    # score
    score = {
        "course_id": "compact_baseline",
        "failed": fail,
        "failures": (["tape_crossing:left_boundary_3"] if fail else []),
        "distance_m": 12.3,
        "max_speed_mps": 0.49,
        "finish_reached": (not fail),
        "speed_check_complete": True,
    }
    sc = String()
    sc.data = json.dumps(score, sort_keys=True)
    put("/igvc_sim/score", sc, 19.0)
    del w  # close/flush

    (run_dir / "final_score.txt").write_text(
        'data: "%s"\n' % json.dumps(score, sort_keys=True).replace('"', '\\"'),
        encoding="utf-8")
    (run_dir / "mission.log").write_text(
        ("mission aborted at waypoint x\n" if fail else "mission complete\n"),
        encoding="utf-8")


def check(name: str, cond: bool) -> bool:
    print(("  OK  " if cond else "  FAIL") + " " + name)
    return cond


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="ar_selftest_"))
    ok = True
    try:
        # clean run
        clean_dir = tmp / "clean"
        clean_dir.mkdir(parents=True)
        write_bag(clean_dir, fail=False)
        m = M.compute_metrics(clean_dir, COURSE)
        print("clean metrics:", json.dumps(
            {k: m[k] for k in ("traversal_time", "breadcrumb", "gradient",
             "pathfootprint_rejects", "backup", "ang_reversals", "stuck_events",
             "time_below_speed", "finish_reached", "violations", "mission_completed",
             "min_course_clear", "pca_first_s")}, default=str))
        ok &= check("score parsed + clean", m["score_loaded"] and not m["failed"]
                    and m["finish_reached"] and not m["violations"])
        ok &= check("mission_completed True", m["mission_completed"] is True)
        ok &= check("traversal_time ~16s",
                    m["traversal_time"] is not None and 15.0 <= m["traversal_time"] <= 17.0)
        ok &= check("breadcrumb==1", m["breadcrumb"] == 1)
        ok &= check("gradient==1", m["gradient"] == 1)
        ok &= check("pathfootprint_rejects==1", m["pathfootprint_rejects"] == 1)
        ok &= check("backup==1", m["backup"] == 1)
        ok &= check("ang_reversals>0", (m["ang_reversals"] or 0) > 0)
        ok &= check("stuck_events>=1 (10-12s gap)", (m["stuck_events"] or 0) >= 1)
        ok &= check("time_below_speed>=1.5", (m["time_below_speed"] or 0) >= 1.5)
        ok &= check("min_course_clear computed +", m["min_course_clear"] is not None
                    and m["min_course_clear"] > 0)
        ok &= check("pca_first_s computed", m["pca_first_s"] is not None)

        # fitness on a 3x clean candidate
        res = F.evaluate_candidate([m, m, m], course="compact_baseline", tier=2,
                                   commit="deadbeef", best_fitness=None)
        print(F.report_card(res, [m, m, m]))
        print(F.result_line(res))
        ok &= check("gate PASS on 3x clean", res["gate"] == "PASS")
        ok &= check("decision KEEP vs no best", res["decision"] == "KEEP")
        ok &= check("fitness is finite", res["fitness"] is not None)

        # failing run -> gate FAIL
        fail_dir = tmp / "fail"
        fail_dir.mkdir(parents=True)
        write_bag(fail_dir, fail=True)
        mf = M.compute_metrics(fail_dir, COURSE)
        ok &= check("fail run detected (violations)", bool(mf["violations"]))
        resf = F.evaluate_candidate([m, mf, m], course="compact_baseline", tier=2)
        ok &= check("gate FAIL on 1 bad run", resf["gate"] == "FAIL"
                    and resf["decision"] == "DISCARD")

        # incomplete run (no score) -> not clean
        inc_dir = tmp / "inc"
        (inc_dir / "bag").mkdir(parents=True)
        mi = M.compute_metrics(inc_dir, COURSE)
        ok &= check("missing bag/score -> INCOMPLETE (run_clean None)",
                    F.run_clean(mi) is None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("RESULT: " + ("ALL PASS" if ok else "SOME FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
