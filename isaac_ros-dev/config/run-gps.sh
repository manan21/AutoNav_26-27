#!/bin/bash
ros2 run gps_handler gps_publisher &
pid_pub=$!

# Coldstart bias: on the first GPS goal accepted by this node, snap
# the EKF's θ_offset to whatever value makes the normal world→odom
# projection of the goal land directly in front of base_link. The
# robot drives toward the real GPS waypoint at the real GPS distance,
# accumulates the displacement that bootstrap_theta needs to refit θ
# from data, and the seed is overwritten by the closed-form fit as
# soon as baseline > BOOTSTRAP_MIN_BASELINE_M. The 45° seed variance
# below is intentionally loose so the EKF's first real heading update
# dominates immediately. One-shot per node — restart re-arms.
ros2 run gps_waypoint_handler gps_handler_node --ros-args \
    -p coldstart_bias_enabled:=true \
    -p coldstart_theta_seed_variance_deg:=45.0 &
pid_handler=$!

trap 'kill -INT "$pid_pub" "$pid_handler" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] GPS"

wait
