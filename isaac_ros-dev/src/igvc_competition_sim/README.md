# IGVC Competition Sim Dynamics Calibration

This package routes simulated robot motion through a calibrated command layer:

`/cmd_vel -> igvc_calibrated_dynamics -> /cmd_vel_gazebo -> Gazebo DiffDrive`

The default calibration file is `config/dynamics_calibration.yaml`. It uses the
May 29 deliberate physical tests for command response and turning dynamics. The
straight-line test's lateral/right drift is explicitly excluded from default
flat-ground tuning because the parking lot cross-slope likely contaminated that
measurement.

Useful commands after building and sourcing the workspace:

```bash
ros2 run igvc_competition_sim igvc_calibration_report --decode-rosbag
ros2 run igvc_competition_sim igvc_dynamics_replay --list
ros2 run igvc_competition_sim igvc_dynamics_replay arc_ladder_1 --ros-args -p use_sim_time:=true
```

`igvc_calibration_report --decode-rosbag` uses `rosbag2_py` in a sourced ROS
environment, or the optional Python `rosbags` package when ROS is unavailable.

Disable the calibrated layer only for A/B testing:

```bash
ros2 launch igvc_competition_sim igvc_competition.launch.py use_calibrated_dynamics:=false
```
