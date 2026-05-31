# Camera line-detection gap — ROOT CAUSE FOUND + FIXED (2026-05-30, branch hailmary)

Resolves UNKNOWNS.md #1 and the CONTEXT.md open question (~L228-250):
"detector emits ZERO line points on EVERY course in sim — is it sim render fidelity or an
x86/CUDA CERIAS bug?"

## ROOT CAUSE (verified) — CUDA architecture mismatch, NOT render fidelity
`autonav_detection/CMakeLists.txt` hardcoded `set(CMAKE_CUDA_ARCHITECTURES 87)` (Jetson
Orin, sm_87 only). This x86 dev/autoresearch host's GPU (RTX 3050 Ti Laptop) is **sm_86**.
A binary with only sm_87 SASS has no kernel image for sm_86, so EVERY CERIAS kernel launch
failed at runtime:
```
[ERROR] [lines]: Kernel launch failed: no kernel image is available for execution on the device
[INFO]  [line_detection_node]: Detected 0 line pixels
```
=> 0 line pixels -> 0 `/line_points`, in the sim AND in offline real-bag replay. The
detector, its config, and the sim camera were never the problem. Prior "emissive tape" /
"tape too dark" hypotheses chased a symptom; the kernel simply never ran.

## FIX (applied) — multi-arch fatbin so the SAME source is canon on robot + dev host
`autonav_detection/CMakeLists.txt` builds for archs **86 87 89** (86 = RTX 30 dev/loop
host, 87 = Jetson Orin robot, 89 = Ada spare). The decisive edit pins the TARGET property,
because colcon/ament seeds a CACHE `CMAKE_CUDA_ARCHITECTURES=52` that overrides a
directory-scope `set()` (and a plain `if(NOT DEFINED)` guard never fires —
`enable_language(CUDA)` pre-defines the var; an sm_52-only build then JIT-falls-back to PTX
and gives a SECOND error here, "PTX compiled with an unsupported toolchain", because nvcc
12.9 PTX > driver 12.8):
```cmake
set_target_properties(line_detector PROPERTIES CUDA_ARCHITECTURES "86;87;89")
```
Faster local-only build: `-DAUTONAV_CUDA_ARCHS=native` (or `=86`). Robot sm_87 SASS
unchanged (87 still in the list).

Verify embedded arches: `cuobjdump <build>/autonav_detection/line_detector | grep 'arch ='`
-> now **sm_86, sm_87, sm_89** (was sm_52 only). VERIFIED.

## VERIFICATION (measured on x86, RTX 3050 Ti, after clean rebuild)
Replay real bag `camera_line_static_canon_diagonal_two_feet` (relaxed freshness gates for
sparse offline replay: `-p max_input_age_ms:=100000 -p max_rgb_depth_delta_ms:=100000
-p tf_use_latest:=true`) through the UNMODIFIED detector:
- kernel-launch errors: **0** (was: every frame).
- CERIAS kernel firing: node log "Detected 9130 / 9423 / 9606 line pixels" per frame
  (was: "Detected 0"); diagnostics **max raw_pixels = 10,728** (was 0).
- 23 `"reason":"updated"` line-point publish events; 38 non-empty `/line_points` msgs.
  (NB: the freshness/sync gates and "processing busy; skipped tick" throttle replay; live
  full-rate streams won't show those.)

## Impact on the autoresearch loop
The documented "0 line points on every course" (CONTEXT.md) was THIS bug. With the rebuilt
detector, tonight's loop will exercise camera line detection in sim for the first time.
**REBUILD on the loop host before running:**
`cd isaac_ros-dev && colcon build --symlink-install --packages-select autonav_detection`.

## Next (now that the detector runs on this host)
1. LIVE-sim confirm: `ros2 launch igvc_competition_sim igvc_competition.launch.py
   line_detection_mode:=camera gazebo_server_only:=true launch_nav:=false` (headless
   Xvfb:1 + NVIDIA GL); confirm `/line_points` populated and `/line_detection/debug/mask`
   shows the tape. THEN, only if sim coverage is short vs the real bags, revisit tape
   appearance (e.g. `<lighting>false</lighting>` on the tape visual). Detector no longer
   the blocker.
2. Offline-replay freshness note: default gates (`max_input_age_ms:250`,
   `max_rgb_depth_delta_ms:500`) drop sparsely-recorded bag frames -> "stale camera/depth
   image". Relax for offline replay only; live sim/robot streams are full-rate.

## Host/agent gotchas
- Run ROS/CUDA UNSANDBOXED (agent Bash sandbox lacks GPU -> cudaErrorNoDevice).
- Orphaned backgrounded ros2/topic-echo/ros2-bag procs hold the shell stdout pipe and
  corrupt later command output; reap detached:
  `setsid bash -c 'pkill -9 -f line_detector; pkill -9 -f "topic echo"; pkill -9 -f "ros2 bag"; pkill -9 -f _ros2_daemon' </dev/null`.
