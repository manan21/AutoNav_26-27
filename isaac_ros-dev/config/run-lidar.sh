#!/bin/bash

sudo ip addr flush dev eno1 && sudo ip addr add 192.168.0.2/24 dev eno1 && sudo ip link set eno1 up

ros2 launch sick_scan_xd sick_multiscan.launch.py \
       	hostname:=192.168.0.1 \
       	udp_receiver_ip:=192.168.0.2 \
	publish_frame_id:="lidar_footprint" \
	tf_publish_rate:=0 \
	laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" \

