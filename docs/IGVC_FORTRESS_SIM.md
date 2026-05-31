# IGVC Fortress Simulation

The Gazebo Fortress / IGVC simulation has moved out of this AutoNav repository
into the standalone `autonav_sim` repository:

```text
https://github.com/blobspire/autonav_sim
```

Use the split workspace layout:

```bash
~/autonav_ws/
  src/
    AutoNav_25-26/   # this robot stack repo, on the branch being tested
    autonav_sim/     # standalone sim repo
```

Build and run from the workspace root:

```bash
cd ~/autonav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

cd src/autonav_sim/igvc_competition_sim
./Run_IGVC_COMPETITION_FORTRESS_TEST.command
```

The sim repo intentionally consumes this AutoNav checkout through ROS package
discovery. Do not copy robot-stack packages back into the sim repo. The robot
side remains here:

- `bringup` provides the robot URDF.
- `slam` provides Nav2 params and behavior trees.
- `autonav_detection` provides camera line / PCA / lidar-line detection.
- `autonav_interfaces` provides messages and actions.
- costmap layers, BT plugins, GPS handler, and controller packages remain in
  this repository.

The old in-tree package `isaac_ros-dev/src/igvc_competition_sim` was removed to
avoid duplicate `igvc_competition_sim` package names when both repos are present
in one colcon workspace.
