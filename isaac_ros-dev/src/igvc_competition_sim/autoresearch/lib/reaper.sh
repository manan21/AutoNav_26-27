#!/usr/bin/env bash
# Reap leftover Gazebo/ROS processes from a sim run so iterations don't leak
# processes or GPU memory. Safe to run before AND after every run. Only targets
# the IGVC sim + nav stack patterns, not unrelated processes.
#
# UNVALIDATED against a live sim on this host (no runnable sim here yet); the
# pattern list mirrors the nodes launched by igvc_competition.launch.py.
set -u

PATTERNS=(
  'gz sim'
  'ign gazebo'
  'parameter_bridge'
  'igvc_competition.launch'
  'navigation_launch'
  'controller_server'
  'planner_server'
  'bt_navigator'
  'behavior_server'
  'smoother_server'
  'velocity_smoother'
  'waypoint_follower'
  'costmap'
  'lifecycle_manager'
  'igvc_course_monitor'
  'igvc_mission_runner'
  'igvc_sensor_harness'
  'igvc_camera_bridge'
  'igvc_calibrated_dynamics'
  'autonav_detection'
  'line_detector'
  'grade_detector'
  'lidar_line_detector'
  'pointcloud_to_laserscan'
  'gps_handler_node'
  'robot_state_publisher'
  'component_container'
)

reap_once() {
  local sig="$1"
  for p in "${PATTERNS[@]}"; do
    pkill -"$sig" -f "$p" 2>/dev/null || true
  done
}

# gz server is hosted by ruby; only kill ruby procs that are clearly gz sim
pkill -INT -f 'gz sim' 2>/dev/null || true

reap_once INT
sleep 2
# escalate for stragglers
still=0
for p in "${PATTERNS[@]}"; do
  if pgrep -f "$p" >/dev/null 2>&1; then still=1; fi
done
if [[ "$still" == "1" ]]; then
  reap_once TERM
  sleep 2
  reap_once KILL
  sleep 1
fi

# bound the ROS 2 daemon discovery cache between runs
ros2 daemon stop >/dev/null 2>&1 || true

# report what (if anything) survived
survivors=""
for p in "${PATTERNS[@]}"; do
  if pgrep -f "$p" >/dev/null 2>&1; then survivors="$survivors $p"; fi
done
if [[ -n "$survivors" ]]; then
  echo "reaper: WARNING survivors:$survivors" >&2
  exit 1
fi
echo "reaper: clean"
exit 0
