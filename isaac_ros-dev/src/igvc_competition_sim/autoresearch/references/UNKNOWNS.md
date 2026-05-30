# UNKNOWNS — flippable assumptions (the loop's idea bank)

Each entry: what is set now + why it is uncertain. The agent generates concrete experiments from these
when obvious wins run out. Tie every flip to a measured metric (CONTEXT.md table). Default to flipping
ONE knob per experiment so keep/discard attributes cleanly.

### A1. Global inflation_radius (0.85) / cost_scaling_factor (4.0)
PCA-obstacle stay-away field. Wider/gentler = paths farther from obstacles but risks closing 5 ft gaps.
Metric: pathfootprint_rejects, breadcrumb, min_course_clear, tight_gaps completion. (goal #1/#3)

### A2. Smac `cost_penalty` (2.0)
Weights the inflation field in A* expansion. Higher = centerline hugs clearance, lower = takes the gap.
Verify tight_gaps still has a route. Metric: plan_inscribed_clear, pathfootprint_rejects. (goal #3)

### A3. Smac `rotation_penalty` (5.0) / `allow_reverse_expansion` (false)
Affects in-place turns and jitter; reverse expansion may help ramp_turns sharp swings. (goal #1)

### A4. MPPI CostCritic `cost_weight` (3.2) / `critical_cost` (300) / consider_footprint (true)
How hard MPPI avoids inflated cost — the main "stop skirting/rejecting paths" controller knob.
Metric: disruptive_aborts, time_below_speed, violations. (goal #1)

### A5. MPPI PathAlign/PathFollow weights (8.0/8.0) + offset_from_furthest
Tracking tightness vs corner-cutting and oscillation. Metric: ang_var, ang_reversals, min_course_clear. (goal #1)

### A6. MPPI vx_max/vx_std/wz_std (0.50/0.25/0.45) + velocity_smoother max_velocity (0.50)
Speed vs chatter — dominant lever for traversal time. Smoother comment warns of start-stop chatter >0.50.
Metric: t_mean, ang_reversals, max_speed (must stay < course max). (goal #2 speed)

### A7. PathFootprintSafe inscribed_threshold/lethal_threshold (254/254), pose_stride (1)
Currently rejects only on LETHAL overlap (inscribed==lethal). Loosening cuts breadcrumb fallback but
risks real clips; the diagnostic is rejects-vs-plan_inscribed_clear correlation. (goal #1 root cause)

### A8. Line-layer local halo: inflation_radius (0.80)/inscribed_radius (0.05)/cost_scaling_factor (1.8)
Tape clearance halo width — too wide closes gaps, too narrow lets corners clip tape. Metric:
min_course_clear vs tape, tight_gaps/sparse_lines completion. (goal #2/#3)

### A9. Line-memory mirror clearing (currently allow_decrease:false) — the C-ii feature
Enable decrease + front cone; cone half-angle/range; whether `overwrite_master` must flip or a
local_mirror_layer.cpp change is needed. Metric: global_clear_events, sparse_lines t_mean/recovery. (goal #4)

### A10. Local line_layer view-gate cone (+/-0.40 rad, 1.2-4.5 m) + observation_persistence_ms (8000)
Must widen toward the real ZED HFOV (1.918862 rad) so FREE cells are produced where the camera sees;
persistence vs premature clearing under EKF drift. Metric: global_clear_events, false line marks. (goal #4)

### A11. breadcrumb_reverse (max_crumbs_per_session 15, lethal_cost_threshold 253, bonus_crumb_after_forward true)
How aggressively the robot reverses. Fix upstream planning (C-iii) first, THEN trim reliance.
Metric: breadcrumb count, stuck_events, t_mean. (goal #1/#2)

### A12. gradient_escape (sample_radius 0.80, cost_threshold 200, timeout 15)
Escape efficacy when boxed in (dense_obstacles/ramp_turns). Metric: gradient count+success, stuck_events. (goal #2)

### A13. line_detector gates (brightness_threshold 220, cluster_min_length_m, max_depth_m 6.0, temporal confirm)
Detection sensitivity vs false lines; affects avoidance AND the FREE signal feeding C-ii clearing.
Metric: line_first_s, false marks, sparse_lines behavior. (goal #2/#4)

### A14. grade_detector (traversable_max_deg 16.7, dbscan_eps 0.3, obstacle ranges)
PCA obstacle/ramp classification — false positives create phantom obstacles -> recovery; misses -> contact.
Metric: pca_first_s, obstacle_contact, ramp behavior. (goal #2)

## Resolved
- **R1. Footprint dims** verified consistent across course RobotSpec / nav2 local+global / URDF
  nav_center_joint (0.225) / BT PathFootprintSafe: half_len 0.545, half_w 0.410, pad 0.050. **FROZEN — do
  not shrink.** Padded scorer box +/-0.595 x +/-0.460 at nav_center. (check_footprint.py)
- **R2. ZED HFOV = 1.918862 rad (~110 deg)**, pitched down 0.349 rad, usable ground range ~5 m — the
  reference for the C-ii / A9 / A10 clearing cone.
- **R3. PathFootprintSafe rejects on LETHAL only** (inscribed_threshold==254), so observed rejects are
  real lethal-cell overlaps -> the fix is wider planning (A1/A2), not loosening the gate.
- **R4. 5 courses feasibility-validated** (padded-robot connectivity) — gross-infeasibility ruled out;
  route difficulty confirmed on first sim baseline.
