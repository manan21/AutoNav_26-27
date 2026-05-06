#!/bin/bash
# Bring up the SICK multiscan LiDAR. Configures eno1, launches sick_scan_xd,
# and prints [GUI_READY] Lidar once /scan_fullframe is actually publishing
# (not merely once the launch process is alive).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

sudo ip addr flush dev eno1 && sudo ip addr add 192.168.0.2/24 dev eno1 && sudo ip link set eno1 up

ros2 launch sick_scan_xd sick_multiscan.launch.py \
        hostname:=192.168.0.1 \
        udp_receiver_ip:=192.168.0.2 \
        publish_frame_id:="lidar_footprint" \
        tf_publish_rate:=0 &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "Lidar" /scan_fullframe \
    --type sensor_msgs/msg/LaserScan --qos sensor --timeout 45

wait "$launchpid"
