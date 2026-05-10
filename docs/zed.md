# ZED 2i Camera

Stereolabs ZED 2i depth camera connected over USB-C. Provides rectified RGB, registered depth, camera intrinsics, and built-in IMU. The ROS2 wrapper is vendored as a **pinned git submodule** — see [Submodule pin](#submodule-pin-zed-ros2-wrapper) below; mismanaging this pin is the single most common way to break the build.

## Bringup

Two equivalent paths:

- **GUI** — click the **Camera** button in the launch panel.
- **Manual** — `./config/run-zed.sh` inside the container.

Both paths invoke `ros2 launch zed_wrapper zed_camera.launch.py` with our overrides, then `sleep 5 && echo "[GUI_READY] Camera"` so the queue advances after a fixed pacing window. The GUI flips the dot green when it sees that sentinel; deadline is 45 s (`hud_node.py:513`) before the device is marked failed but kept running.

## Launch arguments (run-zed.sh)

| Arg | Value | Why |
|---|---|---|
| `camera_model` | `zed2i` | Tells the wrapper which device profile to load |
| `publish_tf` | `false` | URDF + slam_toolbox own all robot TFs; the wrapper would conflict |
| `publish_map_tf` | `false` | Same — slam_toolbox publishes `map → odom` |
| `ros_params_override_path` | `…/install/bringup/share/bringup/config/zed_override.yaml` | Our parameter overlay (see below) |

## Override config (`zed_override.yaml`)

Lives at `isaac_ros-dev/src/bringup/config/zed_override.yaml` and is loaded *after* the wrapper's `common_stereo.yaml` + `zed2i.yaml` defaults, so it wins.

The notable override is the IMU/sensor publish rate:

```yaml
sensors:
  sensors_pub_rate: 380.0
```

The wrapper's default is **100 Hz** (capped). We raise it to **380 Hz** so the EKF gets gyro updates fast enough for tight short-horizon fusion. This was added in commit `8863842a` ("Added yaml overwrite to increase IMU publish rate"). Don't lower it without coordinating with whoever's tuning EKF noise.

## Topics

| Topic | Type | Consumer |
|---|---|---|
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | `autonav_detection` line detector, HUD live view |
| `/zed/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/msg/CameraInfo` | line detector (intrinsics for depth → XYZ projection) |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/msg/Image` | line detector (depth parallax to recover XYZ per pixel) |
| `/zed/zed_node/imu/data` | `sensor_msgs/msg/Imu` | EKF (subject to remap; see `ekf_local.yaml`) |
| `/zed/zed_node/depth/depth_info` | `sensor_msgs/msg/CameraInfo` | depth intrinsics (available; not currently consumed) |

The line detector specifically requires all three RGB-rect/depth/camera-info streams to be time-aligned within `max_rgb_depth_delta_ms: 120` (see `line_detector.yaml`); otherwise it drops the frame.

## TF

The camera link is defined in the URDF (`isaac_ros-dev/src/bringup/description/`) — fixed `base_link → zed_camera_link`, mounted **0.4 m forward** of `base_link` and tilted **~45° downward** (rpy `0 0.785398 0`). Because `publish_tf:=false`, the ZED wrapper does **not** broadcast its own frames — they come from the URDF's `robot_state_publisher`.

## Setup expectations

Everything runs inside the Docker container — you do not need to install the ZED SDK on your laptop. The container ships with the matching SDK and the pinned wrapper builds against it. Your laptop only needs SSH and (optionally) a local ROS2 install for RViz.

---

# Submodule pin (zed-ros2-wrapper)

`isaac_ros-dev/src/zed-ros2-wrapper` is a git submodule pinned to a specific Stereolabs release tag. **Do not** track upstream `master` — Stereolabs advances master to whatever SDK they're currently developing against, and a mismatch between the wrapper code and the ZED SDK installed on the Jetson breaks the colcon build.

## Current pin

| Field | Value |
|---|---|
| Tag | `v5.2.0` |
| SHA | `506e04757fdee442f055ddf280dfd36875732623` |
| Tagged | 2026-02-10 |
| Compatible ZED SDK | 5.1.x and 5.2.x |
| Provides packages | `zed_components`, `zed_wrapper`, `zed_ros2`, `zed_debug` |

The Jetson currently runs ZED SDK **5.1.2**. Tag `v5.2.0` is the latest release that does **not** reference `sl::CustomObjectDetectionProperties::object_tracking_parameters` (that nested struct only exists in SDK 5.2), so it is the newest tag that builds cleanly against 5.1.x while still including the `zed_debug` helper package.

## How the lock works

`.gitmodules` has only the URL — no branch, no tag:

```ini
[submodule "isaac_ros-dev/src/zed-ros2-wrapper"]
    path = isaac_ros-dev/src/zed-ros2-wrapper
    url = https://github.com/stereolabs/zed-ros2-wrapper.git
```

That's intentional. The submodule is pinned by **SHA only** — the parent repo records the exact commit hash in its tree, not a symbolic reference. Branch tracking would defeat the lock. So the only correct way to fetch the pinned wrapper is:

```bash
git submodule update --init isaac_ros-dev/src/zed-ros2-wrapper
```

This reads the SHA from the parent's git index and checks out exactly that commit (`506e047`).

**NEVER run** `git submodule update --remote` — the `--remote` flag tells git to ignore the recorded SHA and fetch the latest commit on whatever branch is configured upstream. With no branch configured here, it defaults to `master` at Stereolabs, which is on the SDK 5.2 ABI and will break the build against the installed 5.1.x headers.

## Verifying your pin

Check your local submodule against the parent's recorded SHA:

```bash
git submodule status isaac_ros-dev/src/zed-ros2-wrapper
```

**Clean** (good):
```
 506e04757fdee442f055ddf280dfd36875732623 isaac_ros-dev/src/zed-ros2-wrapper (v5.2.0)
```

**Drifted** (build will break):
```
+bb9d9cddbc0279dae68d77a0ea647ceea12e44c0 isaac_ros-dev/src/zed-ros2-wrapper
```

The leading `+` means the submodule HEAD is **ahead** of the recorded pin; a leading `-` means it's behind or uninitialized. Either state is a build-breaker waiting to happen.

Recovery is simply re-checking-out the pin from inside the submodule:

```bash
cd isaac_ros-dev/src/zed-ros2-wrapper
git fetch --tags origin
git checkout v5.2.0
cd ../../..
git add isaac_ros-dev/src/zed-ros2-wrapper   # only commit if you intended to bump
```

## When (and how) to bump the pin

Re-pin only when:

1. The ZED SDK on the Jetson is upgraded, **or**
2. A specific bug fix or feature lands upstream that we need.

For an SDK upgrade to 5.2.x, re-pin to `v5.2.2` or newer (those tags target the SDK 5.2 ABI). For an SDK downgrade to 5.0.x, re-pin to `humble-v5.0.0`.

Procedure:

```bash
cd isaac_ros-dev/src/zed-ros2-wrapper
git fetch --tags origin
git checkout <new-tag>           # e.g. v5.2.2
cd ../../..

# Verify the build inside the container before committing:
docker exec -u admin koopa-kingdom bash -lc \
  'cd /autonav/isaac_ros-dev && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select zed_components'

# If clean, commit the new pointer:
git add isaac_ros-dev/src/zed-ros2-wrapper
git commit -m "Pin zed-ros2-wrapper to <new-tag> for SDK <x.y> compatibility"
```

After committing, **update the [Current pin](#current-pin) table above** in the same commit so this doc stays in sync with the actual pin.

## Symptoms of an unintended bump

If someone runs `git submodule update --remote` or hand-checks-out master inside the submodule and commits the parent, the next `colcon build` will fail with errors like:

```
error: 'struct sl::CustomObjectDetectionProperties' has no member named 'object_tracking_parameters'
```

against the SDK 5.1 headers. Recovery: `git checkout v5.2.0` inside the submodule and re-commit the parent.

This has happened before — see commits `a8f7f708` (the unintended bump), `51cd0917` (the fix), and `fb78812c` (the doc update). Don't be the next one.

---

# Further reading

- ZED ROS2 wrapper docs — <https://www.stereolabs.com/docs/ros2/zed-node>
- ZED node parameter reference — <https://www.stereolabs.com/docs/ros2/020_zed-node>
- ZED SDK install (only needed if you're rebuilding the container image) — <https://www.stereolabs.com/docs/installation/linux>
