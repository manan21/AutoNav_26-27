# Message for the Next AutoNav Team

Welcome. This is a snapshot of what we learned, what worked, what bit us, and what we'd change. Treat it as a starting map, not gospel — the course changes, the rules drift, and the codebase will keep moving.

---

# Hardware Recommendation: NX Super (brain) + Nano Super (spine)

**Recommendation: Jetson Orin NX Super (brain) + Jetson Orin Nano Super (spine) — not a single Nano Super.**

A single Nano Super repeats the failure: one board with one thermal budget, already at 80–90% CPU before it throttles, and one fault domain. The split attacks each piece:

- **NX Super (brain):** 8 cores, 16 GB, 102 GB/s, up to 157 TOPS — headroom to run perception/SLAM/Nav2 without saturating, so SLAM keeps mapping and scans/frames stop dropping. Push the detection pipelines onto the GPU so they stay fast enough to catch a line before the robot is already over it. Owns camera, lidar, GPS.
- **Nano Super (spine):** moves motor controller/encoders, Power PCB, Arduino, and screen off the brain — clearing the control lag — and runs the safety layer in a separate fault domain.
- **Active cooling + tuned `nvpmodel`/`jetson_clocks`:** recovering only after minutes of rest is thermal throttling, so without real cooling the new headroom vanishes the moment it heats up.
- **Fix the GPS node:** it was consuming a large share of CPU, which NMEA at a few Hz should never cost — profile it for busy-polling or an over-high update/fusion rate, and move its driver to the spine if it stays hungry.
- **Degradation-aware watchdog:** the spine tracks the brain's topic *rates and latency*, not just liveness — nothing died this year, so a death-only check would never have fired. When scan/SLAM/detection rates fall below threshold, the spine slows or safe-stops until the brain recovers, so the robot never crosses a line it's too throttled to see. RoboteQ command watchdog stays as the dumb bottom layer.

**Result:** a throttle episode becomes a managed slow-and-recover instead of a blind, laggy robot missing lines.

---

## Beyond AutoNav

- **Reusable for the IGVC Self-Drive challenge, not just AutoNav.** The brain/spine platform isn't tied to the AutoNav course — it carries straight into a Self-Drive entry, where the heavier perception load (sign/signal recognition, pedestrian detection, lane keeping) leans even harder on the NX's GPU headroom, and an independent safety layer matters *more*, not less, on a vehicle moving in a road-style environment.
- **The split mirrors how planetary rovers are built.** Rovers on the Moon and Mars pair a capable main computer with independent fault-protection that drops the vehicle into a safe state when something degrades, because a single unrecoverable failure ends the mission. The brain/spine split applies the same principle: an independent layer, in its own fault domain, that keeps the robot safe and controllable when the primary compute falters.

---

## Build notes

- **Inter-board link:** one Ethernet hop between brain and spine (small switch or USB-GbE dongle — the lidar already occupies the NX's onboard RJ45). Bind DDS to that private link; keep it off WiFi.
- **Device partition:** GPS, lidar, camera → NX (brain). Motor controller/encoders, Power PCB, Arduino, screen → Nano (spine). No shared USB devices; share *data* over DDS, not devices.
- **Regeneration:** `respawn`/`systemd` auto-restart for anything that does crash, layered under the degradation-aware watchdog above.
- **Storage:** NX module has no onboard storage — use the existing NVMe SSD.
- **Before buying:** confirm the 16 GB SKU, check Ethernet port count, and verify Isaac ROS supports the kit's JetPack version (6.2 / Ubuntu 22.04 aligns with ROS 2 Humble).

---

# Codebase Recommendations

The hardware story above is half the picture. The other half is the software we left you. Read these before you start ripping things out — most of them encode an incident we already paid for.

## Architecture worth preserving

- **Brain/spine compute split (above) has a software mirror already.** The control node owns the safety layer and publishes `/cmd_vel_monitor`. The perception/planning stack publishes `/cmd_vel`. Keep that boundary clean when you re-platform — don't let the safety node start importing perception types.
- **BT recovery cascade in `bt_nav.xml`.** PlanRecoveryEscape is breadcrumb → gradient_escape → clear+wait. No blind BackUp in the plan-failure path; GoalBender handles "buffer empty + path behind." This shape was validated against the sim's three-behavior flow — preserve it.
- **Point-based planner config.** NavfnPlanner reads a single-cell cost; the 0.41 m inscribed band (from polygon footprint) hard-blocks body overlap, the 0.70 m soft halo prefers wider clearance. If you swap to Smac or RPP (see "Areas to revisit"), reproduce these two bands explicitly — they are load-bearing, not defaults.
- **Costmap paste pattern: global = accumulated paste of local.** No `static_layer`, no `/map_padded`. The planner must respect the local view; this is enforced by construction. If you add a static layer "for convenience," you will regress on the first costmap drift episode and not know why.
- **Line projection LUT.** Camera mount is static, so the pixel→base-frame ground projection is precomputed once and per-frame work collapses to LUT lookup + one matvec. This is what gets you 15 Hz line gathering. Don't "simplify" it back to per-pixel raycasts.
- **Ground-first projection in the line detector.** Raycast to the ground plane *before* trusting ZED depth. This fixed a 2–5 s turn-lag from depth holes on white tape. If you change cameras, redo this step — don't skip it.
- **`nav_center` vs `base_link`.** Nav2 plans from `nav_center` (geometric body center), not `base_link` (drive axle). This is intentional — it produces smoother EKF odom at the cost of corner-cut feedback. Don't "fix" it without understanding which trade you're making.

## Footguns the codebase will hand you

- **Many `/cmd_vel` publishers is by design.** `behavior_server`, `controller_server`, `velocity_smoother`, `control_node`, recoveries — they all write to `/cmd_vel`. This will look like a bug. It isn't. Don't add a mux without understanding what each writer does.
- **Any `/cmd_vel` monitor must tag auto vs. manual.** Joystick output through the control node otherwise looks identical to nav2 misbehaving. You will waste a full day chasing a "nav2 bug" that turns out to be the operator on the stick. Tag the source.
- **AUTO toggle must NOT cancel the nav2 goal.** The operator workflow is "load up a goal, then engage AUTO." Cancelling on AUTO-off makes the robot unusable on the field. There is a memory entry on this; honor it.
- **Forward-only DWB.** The DWB config is intentionally forward-only. Reverse motion happens through explicit recovery behaviors, not through the controller's own search space. Don't widen the search to "improve" anything.
- **No oscillation.** The competition rules and the controller can both produce oscillation. The fix is upstream — tighter angular limits, raised `min_speed_mps`, the stuck-check in BT — not "let it wiggle a little." If you see it wiggling, something's wrong.
- **No PCA threshold changes.** The 16.7° PCA ground/ramp threshold is calibrated against the actual ramp and payload. Changing it requires a full validation run on the real ramp, not a sim run.

## Workflow footguns

- **Jetson is pull-only.** Never `scp` files onto the Jetson, never edit on the Jetson directly, never commit on the Jetson. The GUI's "git sync (hard)" silently destroys local state on every sync — you will lose work. Edit on your laptop, push, pull on the Jetson.
- **Never push without explicit confirmation.** Authorization is per-action, not per-session — even if you pushed five minutes ago.
- **Refactor discipline.** After removing or renaming a variable, grep for stale references and walk every callback path before claiming the change is ready. The control node and BT both have indirect references through string topic names — the compiler will not catch you.

## Areas to revisit (we ran out of time)

- **Costmap drift on repeated traversal.** Local and global costmaps misalign on repeated passes through the same area. Suspected localization drift, not a costmap bug. We did not fix it; keep it in mind when interpreting field behavior. Line memory accumulating old tape cells over multiple loops is a related symptom — does line memory need a decay or clear-on-loop-closure?
- **GPS convergence vs. speed.** At the 0.50 m/s outdoor cap, GPS lag makes the robot "wander like a lost dog." Phase C.2 (0.80 m/s) is gated on `ekf_global` GPS-trust tuning. The dual_ekf_navsat config is the right place to start.
- **GPS node CPU hog.** Same node as above — it shouldn't cost what it costs at a few Hz NMEA. Profile for busy-polling or an over-high fusion rate before just throwing it on the spine.
- **`shogi.urdf` axle offset.** The active URDF has the axle behind body center, which produces off-tracking. `bowser.urdf.xacro` is a separate platform without this offset and was not made active. If you ever swap platforms, this is the file to update.
- **`fix/lines` branch intent.** The name says line-following, but most of the edits there are upstream foundation work — odom, EKF, SLAM TF, the candidate smoother. If you cherry-pick from it, read the diff, not the branch name.
- **`autoresearch_path` line layer fix.** `LineLayer::matchSize()` resizes but doesn't reset background to FREE_SPACE, so old code published high non-lethal cost (94) across the entire map. The fix on `autoresearch_path` initializes default to FREE_SPACE and resets after resize before restamping persisted cells. Land this on main before you start tuning anything else in the costmap stack.
- **RPP vs. MPPI vs. DWB.** `autoresearch_path` swapped to RPP and it tracked curved gap routing better than DWB — but it required the collision gate disabled, which is less conservative than we wanted. Pick this thread back up; the right answer is probably RPP with a properly-tuned collision gate, not "RPP wins."

## What the launch stack is doing

The bringup is staged on purpose (`docs/LAUNCH_STACK.md`): Pre-SLAM → Camera → Lidar → GPS → PCA DETECT → LINE DETECT → SLAM → NAV2 → Power PCB, with a 0.5 s sentinel delay per subsystem. Roughly 30–60 s total. **LIDAR LINE DETECT is opt-in** — not in Run All — and is required for retroreflective tape segments of the course. If the new course removes retroreflective tape, drop it.

## What we wish we'd done earlier

- **Treated thermals as a first-class constraint, not an afterthought.** Active cooling and `nvpmodel`/`jetson_clocks` tuning should have been day-one work, not week-eight work.
- **Built the degradation-aware watchdog before we needed it.** A liveness-only check never fires because nothing dies — things just get slow. Build the rate/latency watchdog before the first long field test.

