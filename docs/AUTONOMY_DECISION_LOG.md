# Autonomy Decision Log

Purpose: preserve design decisions, experiment results, failed approaches, and reversals across agent context windows. Future agents should read this file before changing Nav2, costmaps, perception, behavior trees, simulator geometry, or bag-analysis pass/fail criteria.

This is not a changelog. It is a reasoning log: what was tried, what happened, why the current design exists, and what should not be repeated without new evidence.

## Update Rules

- Add an entry for every meaningful autonomy, simulation, perception, safety, or analysis decision.
- Add failed or reverted approaches. Do not delete them just because they were wrong.
- If a later decision reverses an earlier one, mark the older entry `Superseded` and link or name the newer entry.
- Separate facts from hypotheses. Write "observed", "measured", or "bag shows" only when there is recorded evidence.
- Include enough reproduction detail that another agent can rerun or disprove the conclusion.
- Prefer metrics over impressions: bag path, command, commit, parameter values, clearance, status, and failure mode.
- Keep entries concise. Put deep analysis in a separate doc if needed and link it here.

## Status Labels

- `Accepted`: current design direction.
- `Validated`: accepted and tested with recorded evidence.
- `Rejected`: tried and should not be repeated without a new reason.
- `Superseded`: replaced by a later decision.
- `Open`: plausible but not yet proven.
- `Inconclusive`: test did not isolate the subsystem or evidence was incomplete.

## Entry Template

```md
### YYYY-MM-DD - Short Decision Name

Status: Accepted | Validated | Rejected | Superseded | Open | Inconclusive
Area: planner | controller | costmap | BT | perception | simulation | analysis | operations
Related commits: AutoNav `<hash>`, Sim `<hash>`
Evidence: bag/run path, command, metric, screenshot, or source doc

Decision:
- What changed or what rule should be followed.

Why:
- The technical reason.

Observed result:
- What actually happened in simulation, robot test, or static analysis.

Do not repeat:
- Any dead end this entry is meant to prevent.

Next check:
- The next validation needed, if any.
```

## Current Decisions And Findings

### 2026-05-28 - 5 ft IGVC Passages Must Remain Traversable

Status: Accepted
Area: competition rules, simulation, planner, costmap
Related commits: AutoNav `3ff4d5f3`, Sim `fc884e7`
Evidence: `docs/IGVC_COMPETITION_RULES.md`, official 2026 IGVC rules referenced there

Decision:
- Treat a 5 ft (`1.524 m`) tape-to-obstacle or obstacle-to-obstacle passage as a valid route the robot may need to drive.
- Do not tune the robot or simulator so that a legal 5 ft passage is effectively blocked.
- Gaps narrower than 5 ft can be rejected when another legal route exists.

Why:
- IGVC AutoNav guarantees at least 5 ft of driving space around obstacles in the valid course route.
- The robot should solve the legal route selection problem, not assume tight legal gaps are invalid.

Observed result:
- The canonical simulator now uses a 10 ft nominal lane and a 5 ft tape-to-cone gap for the primary lidar-line regression.

Do not repeat:
- Do not widen the canonical passable gap just to make Nav2 pass.
- Do not use global inflation or footprint padding that mathematically closes a 5 ft passage.

Next check:
- Keep this constraint in future random-obstacle simulation scenarios and physical course layouts.

### 2026-05-28 - Split Hard Keepout From Soft Clearance

Status: Validated
Area: costmap, planner, controller
Related commits: AutoNav `3ff4d5f3`, Sim `fc884e7`
Evidence:
- `/home/cole.guest/autonav-work/lidar_line_runs/through_gap_10ftlane_cost32_20260528_142836`
- `/home/cole.guest/autonav-work/lidar_line_runs/through_gap_10ftlane_cost32_rerun1_20260528_142940`
- `/home/cole.guest/autonav-work/lidar_line_runs/through_gap_10ftlane_cost32_rerun2_20260528_143037`
- `/home/cole.guest/autonav-work/lidar_line_runs/through_gap_10ftlane_cost32_gtpca_20260528_143224`

Decision:
- Use exact tape/cone cells and the real padded robot footprint for hard collision rejection.
- Use wider non-lethal costs to prefer clearance in open lanes.
- Allow Smac and MPPI to traverse soft costs when that is required to use a legal 5 ft passage.

Why:
- A single large hard inflation radius around both sides of a 5 ft gap can close the gap even though the physical route is legal.
- Simply reducing all inflation makes the planner/controller more willing to travel near tape in open lanes.
- The correct model is hard geometry for collision/rule violation, plus soft clearance preference for comfort and robustness.

Observed result:
- Three clean real-PCA runs and one ground-truth-PCA run passed the canonical 10 ft lane / 5 ft gap scenario.
- The final real-PCA reruns had zero executed lethal footprint overlap samples.
- The final ground-truth-PCA run also had zero executed lethal footprint overlap samples.

Do not repeat:
- Do not solve legal-gap failures by only shrinking all cost fields until the robot line-hugs.
- Do not solve line-hugging by making soft clearance into hard global geometry that blocks the 5 ft passage.

Next check:
- Add a wider open-lane regression that verifies the robot stays reasonably centered when there is no need to use the 5 ft minimum passage.
- Add a narrower decoy-gap scenario where the robot prefers the legal 5 ft route.

### 2026-05-28 - Canonical Lidar-Line Sim Geometry Is 10 ft Lane, 5 ft Gap

Status: Validated
Area: simulation
Related commits: AutoNav `3ff4d5f3`, Sim `fc884e7`
Evidence: same four runs listed in "Split Hard Keepout From Soft Clearance"

Decision:
- The canonical lidar-line course uses:
  - left lane tape at `y = +1.524 m`;
  - perpendicular tape ending at `y = -0.13 m`;
  - cone left boundary around `y = -1.654 m`;
  - nominal through-gap centerline around `y = -0.89 m`;
  - through-gap goal near `x = 2.50 m`, `y = -0.89 m`.

Why:
- This matches the IGVC nominal 10 ft track-width framing while preserving a 5 ft minimum passable corridor between tape and cone.
- It forces the stack to solve the competition-relevant case instead of an artificially roomy course.

Observed result:
- Nav2 planned and MPPI drove through the passable corridor in the final real-PCA and ground-truth-PCA validation runs.

Do not repeat:
- Do not move the cone or tape to make the route easier without documenting the reason and keeping a 5 ft regression.
- Do not interpret the old looser geometry as the canonical competition proxy.

Next check:
- Add scenario parameters for randomized cone/tape variants once the fixed canonical case remains stable.

### 2026-05-28 - PathFootprintSafe Should Reject Lethal Footprint Overlap, Not Soft Cost

Status: Validated
Area: behavior tree, costmap, safety
Related commits: AutoNav `3ff4d5f3`
Evidence: final canonical sim runs listed above; installed `bt_nav.xml` includes `PathFootprintSafe`

Decision:
- `PathFootprintSafe` should gate execution on footprint overlap with lethal/raw obstacle cells.
- It should not reject a path solely because the footprint crosses non-lethal soft-cost clearance cells.

Why:
- Soft costs intentionally represent preferences. Treating them as hard failure can block the legal 5 ft passage.
- The behavior tree's job is to prevent unsafe execution, not to forbid every path that enters a high-cost but traversable region.

Observed result:
- The updated BT plugin subscribes directly to raw Nav2 costmap data and validates path footprint overlap against that map.
- Canonical final runs completed without executed lethal footprint overlap.

Do not repeat:
- Do not make the BT safety gate reject arbitrary inflated cost unless the intent is to create a true keepout region and the 5 ft passage remains viable.

Next check:
- Test an intentionally bad goal through tape or cone and confirm the BT gate prevents unsafe execution.

### 2026-05-28 - Global-Plan Lethal Overlap Alone Is Diagnostic

Status: Accepted
Area: analysis, behavior tree, planner
Related commits: AutoNav `3ff4d5f3`
Evidence: `docs/ROS_BAG_ANALYSIS.md`, `scripts/run_lidar_line_bag_analysis.sh`

Decision:
- Use global-plan costmap overlap as a diagnostic signal.
- Use executed footprint overlap with lethal raw costmap cells as the hard pass/fail safety criterion.
- Keep path-through-gap checks as a pass/fail condition for the canonical gap test.

Why:
- Nav2 can publish candidate global plans that the behavior tree later rejects before `FollowPath`.
- Failing a run solely because a rejected candidate plan overlapped lethal cells can confuse planner diagnostics with executed behavior.
- Executed footprint overlap is the better safety outcome measure.

Observed result:
- The standard bag-analysis suite now runs action-result checks, gap routing checks, global-plan diagnostics, and executed-footprint lethal overlap checks.

Do not repeat:
- Do not hide global-plan overlap warnings. They are still valuable for root-cause analysis.
- Do not treat every diagnostic global-plan warning as equivalent to the robot physically clipping tape.

Next check:
- Preserve this distinction when adding multi-goal or randomized-course analyzers.

### 2026-05-28 - Use Direct NavigateToPose Goals For Canonical Tests

Status: Accepted
Area: operations, behavior tree, analysis
Related commits: AutoNav `3ff4d5f3`
Evidence: `docs/LIDAR_LINE_AVOIDANCE_COURSE.md`

Decision:
- Prefer one direct `/navigate_to_pose` action goal for the canonical lidar-line test.
- Do not publish the same goal through both `/goal_pose` and a direct action client.

Why:
- Duplicating goal paths can create overlapping NavigateToPose goals and confusing action-status streams.
- Direct action mode gives an accepted/result stream and still lets the BT/Nav2 stack execute the configured path.

Observed result:
- The simulator test runner and manual live RViz procedure use direct action goals for the canonical through-gap target.

Do not repeat:
- Do not debug apparent abort/succeed mixtures before first checking whether duplicate goals were sent.

Next check:
- Keep robot-side scripts and simulator-side scripts aligned on this convention.

### 2026-05-28 - Do Not Use The Full Test Runner For Long-Lived RViz Visualization

Status: Accepted
Area: operations, simulation
Related commits: Sim `fc884e7`
Evidence: `lidar_line_sim/Run_LIDAR_LINE_ROS_COURSE_TEST.command`

Decision:
- For visualization, launch the live stack with `Run_LIDAR_LINE_ROS_COURSE.command`, then launch RViz with `Run_LIDAR_LINE_ROS_COURSE_RVIZ.command`, then send goals manually.
- Use `Run_LIDAR_LINE_ROS_COURSE_TEST.command` for automated bagged regression runs.

Why:
- The automated test runner launches its own stack, records a bag, sends a goal, runs analysis, and tears the stack down.
- That behavior is correct for regression but wrong when the goal is to keep RViz open for manual inspection.

Observed result:
- Live stack plus RViz VNC allowed inspection while a canonical through-gap NavigateToPose goal completed successfully.

Do not repeat:
- Do not launch the full test command when the user asks to visualize a persistent live stack unless they explicitly want a bagged regression run.

Next check:
- Consider adding a helper command that sends the canonical goal into an already running live stack.

## Open Items To Track

- Add open-lane centering regression: verify soft costs keep the robot away from tape when the lane is wide.
- Add narrow-decoy-gap regression: verify the robot rejects sub-5 ft gaps when a legal route exists.
- Add bad-goal regression: verify the BT safety gate blocks unsafe execution through tape or cone.
- Add randomized obstacle course variants while preserving the fixed 5 ft canonical course.
- Run the same hard/soft clearance design on the physical robot course and record comparable bag metrics.
