#!/bin/bash
# Run the line detector and emit [GUI_READY] LINE DETECT once /line_points
# starts publishing.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/gui_ready.sh"

ros2 run line_detection line_detector --ros-args \
    -p camera_topic:="/zed/zed_node/rgb/color/rect/image" \
    -p depth_camera_topic:="/zed/zed_node/depth/depth_registered" \
    -p camera_info_topic:="/zed/zed_node/rgb/color/rect/camera_info" \
    -p line_points_topic:="/line_points" \
    -p target_frame:="map" \
    -p line_hold_timeout_ms:=0 \
    -p enable_timer:=true &
launchpid=$!
trap 'kill -INT "$launchpid" 2>/dev/null' INT TERM

gui_ready_wait "LINE DETECT" /lines_pointcloud \
    --type sensor_msgs/msg/PointCloud2 --qos sensor --timeout 45

wait "$launchpid"
