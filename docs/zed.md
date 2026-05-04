# ZED Autonav User manual


this document is intended as a knowledge base for anything ZED related for AutoNav 24/25.

---


# Fast Use Guide

To launch the zed node, you can run `zedup` from the ros2_ws directory. This command is an alias for the zed node launch file,
`ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i'`. Remember to source the ROS setup script in each terminal you open.

This node publishes rolling camera data to a variety of topics. The list can be found at:

[https://www.stereolabs.com/docs/ros2/zed-node]

this link also contains other very useful information on the zed node.


---


# features and where to find them

feature exploration with the zed is ongoing. 

Useful features will be documented here, along with links to further documentation

## ROS parameters

The zed camera has a suite of configuration parameters available for the ROS2 wrapper.

The link can be found [here](https://www.stereolabs.com/docs/ros2/020_zed-node)

```
gpu_id - gpu id for computation

camera_flip [bool] - if mounted upside down

pub_resolution ['NATIVE' | 'CUSTOM' | 'OPTIMIZED'] - custom for saving bandwidth. set general.pub_downscale_factor to reduce bandwidth.

pub_frame_rate [int] - set publishing frequency. 

```

Other Parameter topics:
- streaming
- camera image 
- ROI 
- depth 
- odometry and localization
- global localization 
- mapping
- object detection



---

# setup

On the team laptop, I am running cuda 12.6, the '560' nvidia gpu kernel driver (run `nvidia-smi` to see yours)

The ZED SDK at latest version

[https://www.stereolabs.com/docs/installation/linux]

ros2 humble

[https://docs.ros.org/en/humble/index.html]

zed ros2 wrapper

[https://github.com/stereolabs/zed-ros2-wrapper]


these should be all you need to run the zed node.


---


# Submodule pin (zed-ros2-wrapper)

`isaac_ros-dev/src/zed-ros2-wrapper` is a git submodule pinned to a specific Stereolabs release tag. **Do not** track upstream `master` — Stereolabs advances master to whatever SDK they are currently developing against, and a mismatch between the wrapper code and the ZED SDK installed on the Jetson breaks the colcon build.

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

Submodule commits are tracked by SHA inside the parent repo's tree, not by branch or tag. As long as you use:

```
git submodule update --init isaac_ros-dev/src/zed-ros2-wrapper
```

you will always get the SHA recorded by the parent repo (currently `506e047`). **Do not pass `--remote`** — that explicitly bypasses the pin and pulls the latest of whatever branch is configured upstream.

## When to bump the pin

Re-pin the wrapper only when:

1. The ZED SDK on the Jetson is upgraded, **or**
2. A specific bug fix or feature lands upstream that we need.

For an SDK upgrade to 5.2.x, re-pin to `v5.2.2` or newer (those tags reference the SDK 5.2 ABI). For an SDK downgrade to 5.0.x, re-pin to `humble-v5.0.0`.

## Procedure to bump the pin

```bash
cd isaac_ros-dev/src/zed-ros2-wrapper
git fetch --tags origin
git checkout <new-tag>           # e.g. v5.2.2
cd ../../..

# Verify the build inside the container before committing:
docker exec -u admin koopa-kingdom bash -lc \
  'cd /autonav/isaac_ros-dev && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select zed_components'

# If the build is clean, commit the new pointer:
git add isaac_ros-dev/src/zed-ros2-wrapper
git commit -m "Pin zed-ros2-wrapper to <new-tag> for SDK <x.y> compatibility"
```

After committing, **update the table above** in the same commit so this doc stays in sync with the actual pin.

## Symptoms of an unintended bump

If someone runs `git submodule update --remote` or hand-checks-out master inside the submodule and commits, the next `colcon build` will fail with errors like:

```
error: 'struct sl::CustomObjectDetectionProperties' has no member named 'object_tracking_parameters'
```

against the SDK 5.1 headers. Recovery: `git checkout v5.2.0` inside the submodule and re-commit the parent.

