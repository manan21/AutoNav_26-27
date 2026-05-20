#!/bin/bash
ros2 run gps_handler gps_publisher &
pid_pub=$!

# Coldstart bias keeps the published candidate goal
# coldstart_seed_distance_m straight in front of base_link while the
# EKF's θ_offset is still high-variance, so the very first GPS waypoint
# pulls the robot forward (bootstrapping θ via translational
# displacement) instead of being projected with a half-random θ and
# pointing in a random direction.
ros2 run gps_waypoint_handler gps_handler_node --ros-args \
    -p coldstart_bias_enabled:=true \
    -p coldstart_theta_std_threshold_deg:=30.0 \
    -p coldstart_seed_distance_m:=3.0 &
pid_handler=$!

trap 'kill -INT "$pid_pub" "$pid_handler" 2>/dev/null' INT TERM

sleep 0.5
echo "[GUI_READY] GPS"

wait
