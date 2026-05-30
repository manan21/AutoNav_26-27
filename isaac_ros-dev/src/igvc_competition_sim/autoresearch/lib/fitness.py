#!/usr/bin/env python3
"""Reliability gate + speed-weighted fitness + report card (FROZEN harness).

Objective (user decision): SPEED-WEIGHTED, RELIABILITY-GATED.
  - Any run that crosses a line / hits an obstacle / fails to complete => the
    whole candidate is DISCARDED (reliability gate, 3/3 must be clean).
  - Among clean candidates: minimize traversal time (dominant), with minor
    penalties for recovery activations + jitter and a small clearance bonus.

FITNESS = -(T + R_sec + J_sec - C_bonus)   (higher is better; maximize)
  T      = mean traversal time [s]
  R_sec  = W_RECOVERY*(breadcrumb+gradient+backup+spin+clearcostmap)
           + W_PFS*pathfootprint_rejects
  J_sec  = W_REV*ang_reversals + W_VAR*ang_var + W_STUCK*stuck_events
           + W_BELOW*time_below_speed
  C_bonus= W_CLEAR*max(0, min_course_clear)

Weights are deliberate and tunable; calibrate magnitudes in Phase 0 once the
baseline numbers are known. ASCII only.
"""
from __future__ import annotations

import json
from typing import Any

# --- tunable weights (seconds-equivalent) ---
W_RECOVERY = 2.0     # per breadcrumb/gradient/backup/spin/clearcostmap activation
W_PFS = 0.5          # per PathFootprintSafe reject
W_REV = 0.2          # per cmd_vel angular sign reversal
W_VAR = 5.0          # per unit of angular-velocity variance
W_STUCK = 1.0        # per stuck event (>1s nonzero-cmd gap while not at goal)
W_BELOW = 0.1        # per second below speed while not at goal
W_CLEAR = 5.0        # per meter of min course clearance (bonus)
KEEP_EPS = 0.5       # fitness must beat best by this many s-equivalent to KEEP

_RECOVERY_KEYS = ("breadcrumb", "gradient", "backup", "spin", "clearcostmap")


def run_clean(m: dict) -> bool | None:
    """True=clean, False=hard fail, None=indeterminate (INCOMPLETE)."""
    if not m.get("score_loaded"):
        return None
    if m.get("failed"):
        return False
    if m.get("finish_reached") is False:
        return False
    if m.get("violations"):
        return False
    mc = m.get("mission_completed")
    if mc is False:
        return False
    # finish_reached True + no violations + not explicitly aborted => clean
    return bool(m.get("finish_reached"))


def _mean(vals: list[float | None]) -> float | None:
    nums = [float(v) for v in vals if v is not None]
    if not nums:
        return None
    return sum(nums) / len(nums)


def _agg_recovery(runs: list[dict]) -> dict[str, float]:
    out = {}
    for k in _RECOVERY_KEYS + ("pathfootprint_rejects",):
        out[k] = _mean([r.get(k) for r in runs]) or 0.0
    return out


def evaluate_candidate(runs: list[dict],
                       course: str,
                       tier: int,
                       commit: str = "",
                       best_fitness: float | None = None) -> dict:
    """Gate + fitness for one candidate (N repeat runs on a course)."""
    n = len(runs)
    clean_flags = [run_clean(r) for r in runs]
    n_clean = sum(1 for c in clean_flags if c is True)
    n_incomplete = sum(1 for c in clean_flags if c is None)
    n_fail = sum(1 for c in clean_flags if c is False)

    gate = (n > 0 and n_clean == n)
    result: dict[str, Any] = {
        "course": course, "tier": tier, "runs": n, "pass": n_clean,
        "commit": commit, "gate": "PASS" if gate else "FAIL",
        "n_incomplete": n_incomplete, "n_fail": n_fail,
    }

    if n_incomplete >= 2 and not gate:
        result["status"] = "FLAKY"
        result["fitness"] = None
        result["decision"] = "DISCARD"
        return result

    if not gate:
        result["status"] = "DISCARD"
        result["fitness"] = None
        result["decision"] = "DISCARD"
        # surface why
        reasons = []
        for i, (c, r) in enumerate(zip(clean_flags, runs)):
            if c is not True:
                reasons.append(f"run{i + 1}:" + (
                    "INCOMPLETE" if c is None else
                    ",".join(r.get("violations") or ["incomplete/timeout"])))
        result["notes"] = "; ".join(reasons)
        return result

    # gate passed -> compute fitness
    T = _mean([r.get("traversal_time") for r in runs]) or 0.0
    rec = _agg_recovery(runs)
    R_sec = W_RECOVERY * sum(rec[k] for k in _RECOVERY_KEYS) + \
        W_PFS * rec["pathfootprint_rejects"]
    J_sec = (W_REV * (_mean([r.get("ang_reversals") for r in runs]) or 0.0)
             + W_VAR * (_mean([r.get("ang_var") for r in runs]) or 0.0)
             + W_STUCK * (_mean([r.get("stuck_events") for r in runs]) or 0.0)
             + W_BELOW * (_mean([r.get("time_below_speed") for r in runs]) or 0.0))
    clear = _mean([r.get("min_course_clear") for r in runs])
    C_bonus = W_CLEAR * max(0.0, clear) if clear is not None else 0.0

    fitness = -(T + R_sec + J_sec - C_bonus)
    result.update({
        "status": "KEEP_ELIGIBLE",
        "fitness": round(fitness, 3),
        "t_mean": round(T, 2),
        "t_std": round(_std([r.get("traversal_time") for r in runs]), 2),
        "R_sec": round(R_sec, 2),
        "J_sec": round(J_sec, 2),
        "C_bonus": round(C_bonus, 2),
        "min_course_clear": None if clear is None else round(clear, 3),
        **{k: round(v, 2) for k, v in rec.items()},
        "ang_reversals": _mean([r.get("ang_reversals") for r in runs]),
        "ang_var": _mean([r.get("ang_var") for r in runs]),
        "stuck_events": _mean([r.get("stuck_events") for r in runs]),
        "time_below_speed": _mean([r.get("time_below_speed") for r in runs]),
    })
    # keep/discard vs best
    if best_fitness is None or fitness > best_fitness + KEEP_EPS:
        result["decision"] = "KEEP"
    else:
        result["decision"] = "DISCARD"
    return result


def _std(vals: list[float | None]) -> float:
    nums = [float(v) for v in vals if v is not None]
    if len(nums) < 2:
        return 0.0
    mean = sum(nums) / len(nums)
    return (sum((x - mean) ** 2 for x in nums) / (len(nums) - 1)) ** 0.5


def report_card(result: dict, per_run: list[dict]) -> str:
    lines = [f"=== EVALUATE: course={result['course']} tier={result['tier']} "
             f"runs={result['runs']} ==="]
    for i, r in enumerate(per_run):
        c = run_clean(r)
        tag = "COMPLETE" if c is True else ("INCOMPLETE" if c is None else "FAILED")
        viol = ",".join(r.get("violations") or []) or "-"
        lines.append(
            f"run {i + 1}: {tag}  t={r.get('traversal_time')}s viol=[{viol}] "
            f"bc={r.get('breadcrumb')} ge={r.get('gradient')} bu={r.get('backup')} "
            f"sp={r.get('spin')} cc={r.get('clearcostmap')} pfs={r.get('pathfootprint_rejects')} "
            f"rev={r.get('ang_reversals')} var={r.get('ang_var')} stuck={r.get('stuck_events')} "
            f"clear={r.get('min_course_clear')}")
    lines.append(f"GATE: {result['gate']} ({result['pass']}/{result['runs']} clean)  "
                 f"-> {result.get('decision', '?')}  status={result['status']}")
    if result.get("fitness") is not None:
        lines.append(f"fitness={result['fitness']} (t_mean={result.get('t_mean')} "
                     f"R_sec={result.get('R_sec')} J_sec={result.get('J_sec')} "
                     f"C_bonus={result.get('C_bonus')})")
    if result.get("notes"):
        lines.append("notes: " + result["notes"])
    return "\n".join(lines)


def result_line(result: dict) -> str:
    f = result
    def g(k, d="NA"):
        v = f.get(k)
        return d if v is None else v
    return ("RESULT "
            f"course={g('course')} tier={g('tier')} runs={g('runs')} pass={g('pass')} "
            f"gate={g('gate')} status={g('status')} decision={g('decision')} "
            f"fitness={g('fitness')} t_mean={g('t_mean')} "
            f"bc={g('breadcrumb')} ge={g('gradient')} bu={g('backup')} sp={g('spin')} "
            f"cc={g('clearcostmap')} pfs={g('pathfootprint_rejects')} "
            f"ang_var={g('ang_var')} stuck={g('stuck_events')} "
            f"min_clear={g('min_course_clear')} commit={g('commit')}")


if __name__ == "__main__":
    import sys
    runs = json.load(open(sys.argv[1])) if len(sys.argv) > 1 else []
    res = evaluate_candidate(runs, course="?", tier=0)
    print(report_card(res, runs))
    print(result_line(res))
