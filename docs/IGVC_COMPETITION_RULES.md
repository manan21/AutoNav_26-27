# IGVC AutoNav Competition Rules Reference

Last verified: 2026-05-28 against the official IGVC 2026 rules.

Primary sources:

- Official rules page: http://www.igvc.org/rules.htm
- 2026 rules PDF: http://www.igvc.org/2026rules.pdf

Future agents should re-check the current-year official rules before competition, but the constraints below are the baseline for robot and simulation changes in this repo.

## Agent Design Rules

- Do not tune the robot or simulator in a way that makes a legal 5 ft passage impossible. The AutoNav course guarantees at least 5 ft of drivable passage around obstacles, so our stack must be able to plan and drive through a 5 ft gap.
- Do not solve 5 ft passages by making the robot hug tape in open lanes. Use hard collision constraints for true contact risk and soft costs for clearance preference.
- Course boundary lines and internal lines are no-cross hazards. The robot should never intentionally cross white tape to shortcut a plan.
- Obstacles are real hazards. Touching, displacing, running over, or clipping obstacles must be treated as failures in sim and bag analysis.
- The system must use onboard autonomy and live perception. Do not rely on pre-mapped course geometry, course memorization, or offboard positioning aids.
- Keep speed behavior competition-aware: the robot must be able to average above 1 mph where required, must not exceed 5 mph, and must not sit blocked on course for over one minute.
- Preserve safety behavior: autonomous mode, E-stops, and safety-light state are competition requirements, not optional UI details.

## Course Geometry Rules

- Surface: outdoor asphalt pavement.
- Course length: approximately 500 ft.
- Course area: approximately 120 ft wide by 100 ft deep.
- Track width: varies from 10 ft to 20 ft.
- Nominal track width: approximately 10 ft.
- Minimum turn radius: not less than 5 ft.
- Boundaries: continuous or dashed white lines, approximately 3 inches wide, taped on asphalt.
- Obstacles: random barrels/drums in varied colors, plus possible natural obstacles such as trees or shrubs, manmade obstacles such as posts or signs, ramps, and simulated potholes.
- Ramps/inclines: possible, with gradient not exceeding 15%.
- Simulated potholes: 2 ft diameter solid white circles; they must be avoided.
- Course shape: primarily sinusoidal curves with repetitive barrel obstacles.
- Waypoints: waypoint pairs may be provided for No Man's Land and ramp approach guidance.

## Minimum Passage Rule

The critical AutoNav geometry requirement is the 5 ft minimum passage:

- Between a lane line and an obstacle, there must be at least 5 ft of driving space.
- If an obstacle is in the middle of the course, either side of the obstacle should provide 5 ft of driving space.
- If an obstacle is closer to one side of the lane, the other side must provide at least 5 ft of driving space.

Engineering implication:

- A 5 ft gap is not a "too tight to use" gap. It is the minimum valid route the robot may need to find and drive.
- Gaps narrower than 5 ft may appear between random obstacles, tape, and boundaries. The robot does not need to force itself through those if a legal 5 ft route exists.
- The simulator's canonical lidar-line course should keep the tape-to-cone passable gap at 5 ft, or `1.524 m`; do not widen it just to make a planner pass.

## Costmap And Planner Implications

For Nav2, Smac, MPPI, and custom layers:

- Treat exact tape, cones, barrels, potholes, and obstacle surfaces as lethal collision geometry.
- Use the robot's real padded footprint when checking collision. Do not shrink footprint dimensions below the physical robot just to pass a gap test.
- Keep broad clearance preference as traversable soft cost when possible, not as hard keepout that closes legal 5 ft passages.
- A large global inflation radius around both sides of a 5 ft gap can double-count body clearance when the planner already uses the robot footprint. If that inflation is interpreted as hard or near-hard collision, it can make the legal gap disappear.
- Prefer a two-tier model:
  - hard geometry: exact lethal cells plus a small safety band that still leaves 5 ft passages viable;
  - soft clearance: wider costs that bias paths toward the center of open 10 ft lanes and away from tape/obstacles when there is room.
- The behavior-tree path safety gate should reject footprint overlap with lethal cells. It should not reject every soft-cost cell, or it may block legal 5 ft passages.
- Smac and MPPI should be allowed to traverse higher soft costs when the only legal route is a 5 ft passage, while still preferring larger clearance in 10 ft to 20 ft track sections.

Useful clearance math:

- 5 ft = `1.524 m`.
- If the planner footprint width including padding is `W`, the total remaining centerline slack in a 5 ft gap is `1.524 - W`.
- The maximum symmetric hard margin per side is `(1.524 - W) / 2`.
- With the current approximate padded Nav2 footprint width of `0.92 m`, total centerline slack is about `0.604 m`, or only about `0.302 m` per side before discretization, yaw, localization error, and obstacle thickness.
- Therefore, hard global inflation on both sides must be well below that per-side margin, or replaced with soft clearance costs, for the canonical 5 ft passage to remain viable.

## Perception And Simulation Implications

- Tape/line detection must be treated as boundary detection, not decorative lane marking.
- PCA/cone/barrel detection must create obstacle geometry that planning respects.
- Camera-line detection, lidar-line detection, and PCA obstacle detection can have different runtime profiles, but the lidar-only profile must still satisfy the same competition geometry.
- Simulation should include legal 5 ft passages, nominal 10 ft lanes, random obstacle placements, and narrower decoy gaps that the robot can reject when a legal route exists.
- Do not bake course-specific memorized obstacle locations into the autonomy stack. Sim scenarios can be deterministic for regression testing, but robot behavior should still derive obstacles and lines from sensor topics and costmaps.

## Run And Scoring Rules Relevant To Autonomy

- The vehicle is fully autonomous and unmanned during competition.
- Remote human control is not allowed during the run.
- All sensing, control, and computation must be carried onboard.
- Mapping or course-position memorization is not allowed; judges may adjust the course between runs to invalidate memorized maps.
- Six minutes are allowed for AutoNav course driving.
- The run can end when the vehicle finishes, a judge E-stops, the team E-stops, the vehicle fails to start in time, blocks traffic, loses payload, leaves the course, or is too slow.
- If no vehicle finishes, scoring is based on adjusted distance traveled.
- Boundary crossing, internal-line crossing, obstacle contact/displacement, careless driving, blocking traffic, and team E-stop can reduce score or end the run.
- Crossing internal lines is judged as an E-stop end-of-run event with penalty.
- Stopping on course for over one minute is blocking traffic and ends the run.

## Vehicle And Safety Rules Relevant To Software

- Vehicle dimensions:
  - length: 3 ft minimum, 7 ft maximum;
  - width: 2 ft minimum, 4 ft maximum;
  - height: 6 ft maximum, excluding E-stop antenna.
- Speed:
  - average speed must be above 1 mph;
  - the first 44 ft includes a 1 mph minimum-speed check;
  - maximum speed is 5 mph and must be hardware governed.
- Mechanical E-stop:
  - red push-to-stop button;
  - at least 1 inch diameter;
  - center rear of the vehicle;
  - 2 ft to 4 ft above ground;
  - hardware based, not software controlled;
  - must quickly bring the vehicle to a complete stop.
- Wireless E-stop:
  - effective for at least 100 ft;
  - hardware based, not software controlled;
  - held by judges during performance events.
- Safety light:
  - solid when vehicle power is on;
  - flashing in autonomous mode;
  - returns to solid when leaving autonomous mode.
- Payload:
  - 20 lb payload;
  - approximately 16 in x 8 in x 8 in;
  - securely mounted;
  - loss of payload ends the run.
- Tactile sensors are not allowed for the AutoNav run.

## Qualification Behaviors

The autonomy stack must be able to demonstrate:

- lane following;
- obstacle avoidance;
- waypoint navigation to a single 2 m waypoint while navigating around an obstacle;
- autonomous-mode operation during E-stop, minimum-speed, lane-following, obstacle-avoidance, and waypoint checks.

The waypoint-navigation qualification must be integrated into the original autonomous software. It should not be a separately reconfigured special-case behavior.

## Required Regression Tests

Robot and simulation changes that affect planning, costmaps, line detection, obstacle detection, controller behavior, or behavior-tree safety should preserve these tests:

- Canonical lidar-line course with a 5 ft tape-to-cone gap: global path must route through the gap without lethal footprint overlap, and MPPI must drive it without clipping tape or cone.
- Open 10 ft lane: global and local behavior should prefer reasonable clearance and should not skim tape when there is ample space.
- Center obstacle in lane: planner should find either valid 5 ft side passage.
- Obstacle near one side: planner should prefer the opposite side with at least 5 ft passage.
- Narrow decoy gap below 5 ft: planner may reject the gap when another legal route exists.
- Bad goal through tape or obstacle: behavior-tree safety checks should prevent unsafe execution.
- Recovery behavior: recoveries must not back or turn the robot into tape, cones, barrels, potholes, or course boundaries.

See also `docs/LIDAR_LINE_AVOIDANCE_COURSE.md` for the current indoor 5 ft tape-to-cone regression course.
