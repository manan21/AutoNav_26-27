#!/usr/bin/env python3
"""auto_camera build-loop evaluation entry point (FROZEN harness).

Runs a candidate config N times on a course in the Gazebo sim, scores each run,
applies the reliability gate + speed-weighted fitness, prints a report card and
a machine RESULT line, and appends a row to results/run_log.tsv.

Modes:
  python3 evaluate.py --course tight_gaps --runs 3 --tier 2 [--commit SHA] \
      [--description "..."] [--best-fitness F] [--timeout 300]
  python3 evaluate.py --score-existing DIR1 DIR2 ...   # re-score dirs, no sim

The sim-run path requires a runnable sim env (see references/CONTEXT.md). The
scoring/aggregation/logging path is exercised by --score-existing and the
selftest, and is validated; the live sim-run path is UNVALIDATED on this host.
ASCII only.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LIB = HERE / "lib"
sys.path.insert(0, str(LIB))
import metrics as M          # noqa: E402
import fitness as F          # noqa: E402

COURSES = HERE / "courses"
WORLDS = COURSES / "worlds"
RESULTS = HERE / "results"
RUN_LOG = RESULTS / "run_log.tsv"

LOG_COLS = [
    "iso_time", "commit", "tier", "course", "runs", "pass", "gate", "status",
    "fitness", "t_mean", "t_std", "viol_total", "completion_rate", "breadcrumb",
    "gradient", "backup", "spin", "clearcostmap", "pathfootprint_rejects",
    "followpath_disruptive_aborts", "compute_aborts", "ang_var", "ang_reversals",
    "stuck_events", "time_below_speed", "min_course_clear", "executed_lethal_clear",
    "plan_inscribed_clear", "line_first_s", "line_max_cells", "pca_first_s",
    "global_clear_events", "notes", "description",
]


def _course_paths(course: str) -> tuple[Path, Path]:
    return COURSES / f"{course}.yaml", WORLDS / f"{course}.sdf"


def _run_sim(course: str, run_dir: Path, timeout: int) -> None:
    yaml, world = _course_paths(course)
    if not yaml.is_file() or not world.is_file():
        raise FileNotFoundError(f"course/world missing for '{course}': {yaml}, {world}")
    cmd = ["bash", str(LIB / "run_one.sh"),
           "--course-yaml", str(yaml), "--world", str(world),
           "--run-dir", str(run_dir), "--timeout", str(timeout)]
    subprocess.run(cmd, check=False)


def _g(v):
    return "NA" if v is None else v


def _append_log_row(result: dict, per_run: list[dict], description: str) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    if not RUN_LOG.exists():
        RUN_LOG.write_text("\t".join(LOG_COLS) + "\n", encoding="utf-8")
    viol_total = sum(len(r.get("violations") or []) for r in per_run)
    row = {
        "iso_time": _dt.datetime.now().isoformat(timespec="seconds"),
        "commit": result.get("commit", ""),
        "tier": result.get("tier"),
        "course": result.get("course"),
        "runs": result.get("runs"),
        "pass": result.get("pass"),
        "gate": result.get("gate"),
        "status": result.get("status"),
        "fitness": result.get("fitness"),
        "t_mean": result.get("t_mean"),
        "t_std": result.get("t_std"),
        "viol_total": viol_total,
        "completion_rate": (round(result.get("pass", 0) / result["runs"], 3)
                            if result.get("runs") else 0),
        "breadcrumb": result.get("breadcrumb"),
        "gradient": result.get("gradient"),
        "backup": result.get("backup"),
        "spin": result.get("spin"),
        "clearcostmap": result.get("clearcostmap"),
        "pathfootprint_rejects": result.get("pathfootprint_rejects"),
        "followpath_disruptive_aborts": None,
        "compute_aborts": None,
        "ang_var": result.get("ang_var"),
        "ang_reversals": result.get("ang_reversals"),
        "stuck_events": result.get("stuck_events"),
        "time_below_speed": result.get("time_below_speed"),
        "min_course_clear": result.get("min_course_clear"),
        "executed_lethal_clear": None,
        "plan_inscribed_clear": None,
        "line_first_s": _mean_of(per_run, "line_first_s"),
        "line_max_cells": None,
        "pca_first_s": _mean_of(per_run, "pca_first_s"),
        "global_clear_events": _mean_of(per_run, "global_clear_events"),
        "notes": result.get("notes", ""),
        "description": description,
    }
    line = "\t".join(str(_g(row.get(c))) for c in LOG_COLS)
    with RUN_LOG.open("a", encoding="utf-8") as fh:
        fh.write(line + "\n")


def _mean_of(per_run: list[dict], key: str):
    vals = [r.get(key) for r in per_run if r.get(key) is not None]
    return round(sum(vals) / len(vals), 3) if vals else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--course")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--tier", type=int, default=1)
    ap.add_argument("--timeout", type=int, default=300)
    ap.add_argument("--commit", default="")
    ap.add_argument("--description", default="")
    ap.add_argument("--best-fitness", type=float, default=None)
    ap.add_argument("--run-root", default=str(RESULTS / "runs"))
    ap.add_argument("--keep-bags", action="store_true",
                    help="do not prune bags after scoring")
    ap.add_argument("--score-existing", nargs="+", default=None,
                    help="score existing RUN_DIRs instead of running the sim")
    args = ap.parse_args()

    per_run: list[dict] = []
    course = args.course or "unknown"

    if args.score_existing:
        yaml, _ = _course_paths(course) if args.course else (None, None)
        for d in args.score_existing:
            per_run.append(M.compute_metrics(d, str(yaml) if yaml else None))
    else:
        if not args.course:
            print("ERROR: --course required (or use --score-existing)", file=sys.stderr)
            return 2
        yaml, _world = _course_paths(course)
        run_root = Path(args.run_root)
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        for k in range(args.runs):
            run_dir = run_root / f"{course}_{stamp}_run{k + 1}"
            run_dir.mkdir(parents=True, exist_ok=True)
            print(f"--- {course} run {k + 1}/{args.runs} -> {run_dir} ---")
            _run_sim(course, run_dir, args.timeout)
            m = M.compute_metrics(run_dir, str(yaml))
            per_run.append(m)
            # prune heavy bag unless failing/keep
            if not args.keep_bags and F.run_clean(m) is True:
                bagp = run_dir / "bag"
                if bagp.is_dir():
                    import shutil
                    shutil.rmtree(bagp, ignore_errors=True)

    result = F.evaluate_candidate(per_run, course=course, tier=args.tier,
                                  commit=args.commit, best_fitness=args.best_fitness)
    print(F.report_card(result, per_run))
    print(F.result_line(result))
    _append_log_row(result, per_run, args.description)
    # exit code: 0 if KEEP, 1 if DISCARD/FLAKY (lets a shell loop branch)
    return 0 if result.get("decision") == "KEEP" else 1


if __name__ == "__main__":
    raise SystemExit(main())
