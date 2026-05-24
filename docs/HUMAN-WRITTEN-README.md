# WELCOME TO THE AUTONAV REPO!

## This documentation aims to be brief for human readers to provide a high level overview of the robot and its corresponding repo as a whole.

Firstly... what even is our robot? Think of the robot as a central core (Nvidia Jetson Orin Nano) connected to many sensors and devices (GPS, LiDAR, Camera, Encoders, Power Monitoring, and Motor Controller).

Last Edited - 05/07/2026
By: Nathan Fikes
With: Format and Grammar assisted by CLAUDE.

---

- [WELCOME TO THE AUTONAV REPO!](#welcome-to-the-autonav-repo)
  - [This documentation aims to be brief for human readers to provide a high level overview of the robot and its corresponding repo as a whole.](#this-documentation-aims-to-be-brief-for-human-readers-to-provide-a-high-level-overview-of-the-robot-and-its-corresponding-repo-as-a-whole)
- [The Core (Jetson) requirements](#the-core-jetson-requirements)
- [The Sensors and what they provide](#the-sensors-and-what-they-provide)
  - [GPS](#gps)
  - [SICK Lidar](#sick-lidar)
  - [Zed2i Camera](#zed2i-camera)
  - [Encoders](#encoders)
  - [Power Monitoring](#power-monitoring)
- [ROS2 Framework](#ros2-framework)
  - [How does the core speak with all these sensors?](#how-does-the-core-speak-with-all-these-sensors)
  - [Where are all these Nodes and Topics?](#where-are-all-these-nodes-and-topics)
- [Subsystems](#subsystems)
  - [Camera line detection (brief synopsis)](#camera-line-detection-brief-synopsis)
  - [Lidar obstacle detection (brief synopsis)](#lidar-obstacle-detection-brief-synopsis)
  - [Encoder localization (brief synopsis)](#encoder-localization-brief-synopsis)
  - [Self-Correcting GPS (brief synopsis)](#self-correcting-gps-brief-synopsis)
  - [Power Monitoring PCB (brief synopsis)](#power-monitoring-pcb-brief-synopsis)
- [Autonomy](#autonomy)
- [Drift](#drift)
  - [EKF or Extended Kalman Filter (GPS, IMU, ODOM) and SLAM LiDAR scan matching](#ekf-or-extended-kalman-filter-gps-imu-odom-and-slam-lidar-scan-matching)
  - [SOC empirical curve anchoring during idle (Power PCB)](#soc-empirical-curve-anchoring-during-idle-power-pcb)
- [Software Workflow](#software-workflow)

---

# The Core (Jetson) requirements

What does the team expect our "central core" to do? This is the brains of our robot, and we expect it to do a lot — like *autonomous* a lot.

- It must read data from all the sensors and devices.
- Make real-time decisions on what to do with that data (measured in Hz).
- It must provide the software environment necessary for each of the devices to operate.
- The core must be easy to make changes to.
- It must support the load of a massive amount of calculations.

# The Sensors and what they provide

Our sensors allow the robot to perceive the environment and make those necessary decisions. Let's go over each one so you are familiar with what their task is:

## GPS

The GPS allows the robot to get ground-truth data about the robot's Latitude \[°\] and Longitude \[°\] location. It is necessary because it allows the robot to get to GPS waypoints planted on the course.

## SICK Lidar

On the robot is a 3D LiDAR that provides the robot with a 3D point cloud. This is necessary to allow the robot to avoid obstacles, especially physical ones that are dynamic that the camera might not catch.

## Zed2i Camera

The depth camera on the robot can detect white lines or white potholes and provide them to the robot. This is necessary as the robot must stay within white bounds during the competition, and something like a LiDAR can't detect colors.

## Encoders

Encoders allow the robot to track its X \[m\], Y \[m\], θ \[°\] local transformation. Think of the robot as an ant crawling in a petri dish — this allows the robot to have much more precise localization than what any of the previous sensors can provide.

## Power Monitoring

On the robot is a special PCB that measures V \[V\], I \[A\], P \[W\], and state of charge \[%\]. These metrics are necessary for enabling the robot to function at full potential, or for telling the team when the battery needs to be recharged.

---

# ROS2 Framework

## How does the core speak with all these sensors?

Our solution is ROS2 Humble. Think of ROS2 (Robot Operating System 2) as a special pipe between each sensor. We have a bunch of sensors talk to each other through **Topics** using what are called **Nodes**. As long as the data type across topics is equal, programs of any language (C++, Python, Java, etc.) can seamlessly talk to each other.

## Where are all these Nodes and Topics?

All of our systems live in what are called **Packages**, and each package holds a set of Nodes to run or things to use. Right now we have **22** of these packages.

You can find these packages by looking at `isaac_ros-dev/src/`, where each package is a folder. For instance, inside the **control package**, `isaac_ros-dev/src/control/src/control.cpp` is a ROS2 node. It is self-explanatory that this node helps control the robot. You can always tell a file is a node as long as you see `class [NodeName] : public rclcpp::Node` at the top.

What are all our packages? Here is a list of them:

- **autonav_automated_testing** — Allows the robot to gather and record test data.
- **autonav_detection** — Houses any detection nodes like line detection or object detection.
- **autonav_electrical_publisher** — Has a node that can talk to the external Power Monitor PCB.
- **autonav_interfaces** — Defines all the message formats and services for the project.
- **autonav_sim** — Has all the assets for a full Gazebo simulation of the robot.
- **autonav-gui-hud** — Has a special node that can interact with a GUI system on the robot.
- **bringup** — Robot orchestration like launch files and the description of the robot in 3D space.
- **control** — Any node that has to do with robot control, like autonomy, control, or motor controlling.
- **custom_behavior_tree_plugins** — A place for creating custom behavior trees that activate when the robot is stuck.
- **gps_handler** — Talks to the GPS to get GPS data to the robot.
- **gps_waypoint_handler** — Allows the robot to navigate to GPS waypoints by translating them into local coordinates.
- **line_layer** — Adds lines to a costmap so the robot will avoid them like obstacles.
- **map_padder** — Pads the costmap whenever the robot, path, goal, or obstacles are placed outside the current map.
- **odom_handler** — Helps build odometry values, positions, and heading from raw encoder readings.
- **pointcloud_to_laserscan** — Helps smush 3D scans of point clouds to a 2D surface for a costmap.
- **sick_scan_xd** — Talks to the SICK MultiScan LiDAR.
- **sim** — An older simulation package that hasn't been cleaned up.
- **slam** — Simultaneous localization and mapping, which allows the robot to exist in a virtual environment.
- **zed_components** — Helps get the ZED camera SDK into ROS2 components we can use.
- **zed_debug** — Helps with debugging the camera.
- **zed_ros2** — A meta package used to help install all dependent packages into a system related to ZED.
- **zed_wrapper** — Provides a way to launch the camera.

---

# Subsystems

How is the robot autonomous? What is it doing to be this way?

Let's first approach this from the detection side — how are we identifying things we shouldn't bump into?

First, let's describe what a costmap is. A **COSTMAP** is a 2D surface containing regions from 0% to 100%. A path planner can use this map to create a path of least cost but lowest distance, using optimization techniques. Whenever we say something is added to the costmap, it means this.

## Camera line detection (brief synopsis)

1. We first take an image.
2. Mask the pixels that are above a certain brightness.
3. Apply a per-pixel kernel that will only keep that pixel if the window has:
   - an average brightness above a threshold
   - low brightness standard deviation
4. Use depth parallax to get the XYZ of that pixel and transform it to the robot frame.
5. Do this for all points; when done, these can be added to a costmap.

## Lidar obstacle detection (brief synopsis)

1. Allow the LiDAR to take a 360° theta and 7° to −35° azimuth scan of the surroundings.
2. Chop off the back 180° that contains the robot.
3. Take a point and its neighbors and add them to a covariance matrix.
4. Eigendecompose this covariance matrix for the smallest eigenvalue and find the corresponding eigenvector.
5. Compare that vector with the normal vector — if it is above a threshold, add it to the costmap.

Next: how are we getting to waypoints? What is the robot doing to know where it is and when it has reached a waypoint?

## Encoder localization (brief synopsis)

1. Fully define the robot's wheel radius and track.
2. Use the `x = rθ` formula to get the distance traveled by each wheel.
3. Use the 2-DoF differential drive formulas to get overall distance and heading:
   ```
   d  = (dL + dR) / 2
   Δθ = (dR − dL) / track
   ```
4. Discrete integration over time to get the real values:
   ```
   x = x + d · cos(θ + Δθ/2)
   y = y + d · sin(θ + Δθ/2)
   θ = θ + Δθ
   ```
5. Use these to tell where the robot is in its frame, to see if it is within range of a goal.

## Self-Correcting GPS (brief synopsis)

1. The robot only has access to its own GPS position and local coordinates.
2. Assume there is some imaginary plane with north assumed to be where the robot is facing, anchored at some fixed datum point.
3. Place a candidate goal in the frame of the imaginary plane corresponding to the ENU projection, assuming that the plane's north is True.
4. Allow the robot to drive in whatever direction it would like to.
5. Gather four headings, each pair in its own frame of reference.
6. Take the robot's GPS heading sampled from movement and the GPS heading assuming movement towards the goal exactly. Angle in GPS space measured CCW from north.
7. The heading in that imaginary frame (assisted by the robot's local coordinates) and the angle to the candidate goal measured CCW from fake north.
8. According to the DOF of the system, if R is known, then if theta is equal, the position of the candidate goal and the real GPS goal are equal.
9. Collapse the error in theta with successive iterations.

Additionally, you might be wondering how we are calculating voltage, current, power, and battery level?

## Power Monitoring PCB (brief synopsis)

1. Have a pulse-rated shunt resistor in the high-current path. Measure the voltage drop.
2. Calculate the total current flowing through that resistor using Ohm's law.
3. The ADC on the INA226 chip directly measures bus voltage; multiplying that with current gives power.
4. Have an empirical discharge curve (measured in the lab) used to seed an initial state of charge for the battery. Have things like Peukert filtering and load compensation for accurate seeding.
5. Integrate the current vs. time at a reasonable frequency and subtract it from the capacity of the battery.

---

# Autonomy

The robot uses both obstacles and waypoints inside a costmap, and it will attempt to plan a path. To ensure we stay away from obstacles, we inflate the costmap around each obstacle point (whether from lines or LiDAR).

- SLAM creates the map for all things.
- NAV2 uses Dijkstra to create an optimized path (updates in real time).
- A controller server commands the robot to follow that path.
- A behavior tree triggers when the robot is stuck or has made insignificant progress.
- A ROS2 action server monitors until the robot reaches that goal, and can add another goal in a sequence.

---

# Drift

The bane of all sensors is drifting. How are we handling this problem?

## EKF or Extended Kalman Filter (GPS, IMU, ODOM) and SLAM LiDAR scan matching

This filter compares data points from three sensors (the IMU is inside the camera) to ensure minimal drift. It mainly focuses on ODOM, using both GPS and IMU.

- IMU gyros' yaw and pitch rates assist ODOM velocities.
- SLAM scan-matches LiDAR against the map and corrects ODOM in the MAP frame.
- GPS helps lock down ODOM drift.

## SOC empirical curve anchoring during idle (Power PCB)

Coulomb counting is prone to drifting because it integrates over time.

- When the robot is at rest (i.e., not much transient current draw), we can seed a new SOC (State of Charge).

---

# Software Workflow

Our team has developed a pretty streamlined workflow that we think might help the next team keep the codebase nice and tidy.

- **Branch-based workflow** — create a branch off of `main` and make all your edits there.

  - **Naming conventions:**
    - `fix/<short-name>` — any fixes to a specific system; the fixes are in the context of that system.
    - `feature/<short-name>` — designated for any branch that will add a new feature to the system.
    - `test/<short-name>` — branches that are purely for testing and can be disposed of quickly or merged in somewhere.

    You can make the branch using `git switch -c <branch-name> main` after having pulled in `main`.

  - Branches are typically branched off of `main`, but they could occasionally be branched off other branches. This should be avoided though, as it is better to keep branches separate and not tangled. If you need something from another branch, just merge that branch into yours: `git switch <my-branch> && git merge <source-branch>`.

  - Branches will sometimes become old, in which case it is common to merge `main` into that branch. This helps at PR time when you need a branch that merges in cleanly. It is much safer to catch merge conflicts in the branch than at deployment in `main`.

- **Never push directly into `main`.** Create a PR from your branch once you think everything has been tested and is ready for deployment. PRs are best set up on the GitHub website.

  - Ensure everything in that branch works before posting a PR by doing lab testing.
  - PRs will only be able to be merged into `main` after at least one reviewer has submitted an approval.
  - After a PR has been merged in, you are free to delete the branch you were working on with `git branch -d <branch-name>`.

- **Agentic code development.** If you want to use AI to help code, please follow these rules.

  - Always have AI work in your branch. Never allow it to spawn agents on their own branches.

  - If big changes need to be done, do it this way:

    1. **Sandbox** — Create a local folder on your system and copy the repo folder over as "reference," or add the repo as context. You can create all sorts of simulations over there.
    2. **Plan** — Allow AI to churn a design through spawning multiple agents to dig at the "reference" and have them write an implementation plan. You design the intent.
    3. **House rules** — Ensure both a `RULES.md` and your AI of choice's behavior document exist in that local system, to streamline your local progress. The rules preserve the design intent.
    4. **Apply** — After agents have written an `IMPLEMENTATION_PLAN.md`, have a separate AI instance apply it to a branch in the repo.
    5. **Review** — Please read over everything before testing.

  - If AI writes `README.md` files, try to skim over them to ensure continuity, or ask AI to refine them.
