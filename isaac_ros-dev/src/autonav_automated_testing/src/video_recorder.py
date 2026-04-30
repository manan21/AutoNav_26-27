#!/usr/bin/env python3
"""
video_recorder.py — ROS2 node that records camera thumbnails and LiDAR
bird's-eye-view as MP4 files alongside the DAQ CSV.

Subscribes to:
    /data/toggle_collect  (std_msgs/Bool)  — start/stop, same signal as CSV
    /zed/zed_node/rgb/color/rect/image  (sensor_msgs/Image) — camera
    /scan  (sensor_msgs/LaserScan) — LiDAR

Outputs (per recording session):
    {csv_stem}_camera.mp4       640x360 @ 30fps
    {csv_stem}_lidar_bev.mp4    480x480 @ 30fps
"""

import glob
import math
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

LOG_DIR = '/autonav/logs'
CAM_WIDTH = 640
CAM_HEIGHT = 360
LIDAR_SIZE = 480
CAM_FPS = 15
LIDAR_FPS = 30


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self._bridge = CvBridge()

        # Latest cached messages from subscriptions (converted lazily in timer)
        self._latest_img_msg = None
        self._latest_scan = None

        # VideoWriter handles
        self._cam_writer = None
        self._lidar_writer = None

        # Recording timers
        self._cam_timer = None
        self._lidar_timer = None

        self._recording = False

        # Online flags — log once when first message arrives
        self._camera_online = False
        self._lidar_online = False

        # Subscriptions (always active so frames are cached)
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            self._camera_cb,
            SENSOR_QOS,
        )
        self.create_subscription(
            LaserScan,
            '/scan_fullframe',
            self._scan_cb,
            SENSOR_QOS,
        )
        self.create_subscription(
            Bool,
            '/data/toggle_collect',
            self._toggle_cb,
            10,
        )

        # Publish recording events to /data/dump so they appear in the CSV
        self._dump_pub = self.create_publisher(String, '/data/dump', 100)

        self.get_logger().info('VideoRecorder ready — waiting for toggle_collect')

    # -- Subscription callbacks ------------------------------------------------

    def _camera_cb(self, msg: Image):
        if not self._camera_online:
            self._camera_online = True
            self.get_logger().info('Camera record online')
        # Store raw message — conversion happens in the write timer to avoid
        # blocking the callback (cv_bridge on 1080p is expensive on Jetson)
        self._latest_img_msg = msg

    def _scan_cb(self, msg: LaserScan):
        if not self._lidar_online:
            self._lidar_online = True
            self.get_logger().info('LiDAR record online')
        self._latest_scan = msg

    def _toggle_cb(self, msg: Bool):
        if msg.data and not self._recording:
            self._start_recording()
        elif not msg.data and self._recording:
            self._stop_recording()

    # -- Recording lifecycle ---------------------------------------------------

    def _find_csv(self):
        """Find the most-recent t000_*.csv inside per-run subdirectories of LOG_DIR.
        Returns (directory, stem) or (None, None)."""
        pattern = os.path.join(LOG_DIR, 't000_*', 't000_*.csv')
        files = sorted(glob.glob(pattern), key=os.path.getmtime)
        if not files:
            return None, None
        csv_path = files[-1]
        directory = os.path.dirname(csv_path)
        stem = os.path.splitext(os.path.basename(csv_path))[0]
        return directory, stem

    def _make_writer(self, path, size, fps):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        return cv2.VideoWriter(path, fourcc, fps, size)

    def _start_recording(self):
        run_dir, stem = self._find_csv()
        if stem is None:
            self.get_logger().warn('No t000_*.csv found in logs — cannot name videos')
            return

        cam_path = os.path.join(run_dir, f'{stem}_camera.mp4')
        lidar_path = os.path.join(run_dir, f'{stem}_lidar_bev.mp4')

        self._cam_writer = self._make_writer(cam_path, (CAM_WIDTH, CAM_HEIGHT), CAM_FPS)
        self._lidar_writer = self._make_writer(lidar_path, (LIDAR_SIZE, LIDAR_SIZE), LIDAR_FPS)
        self._recording = True

        self._cam_timer = self.create_timer(1.0 / CAM_FPS, self._write_camera_frame)
        self._lidar_timer = self.create_timer(1.0 / LIDAR_FPS, self._write_lidar_frame)

        self.get_logger().info(f'Recording started → {cam_path}, {lidar_path}')

        # Publish actual recording start events to the CSV stream
        msg = String()
        msg.data = '/recording/camera,event,START'
        self._dump_pub.publish(msg)
        msg.data = '/recording/lidar,event,START'
        self._dump_pub.publish(msg)

    def _stop_recording(self):
        # Publish actual recording stop events to the CSV stream
        msg = String()
        msg.data = '/recording/camera,event,STOP'
        self._dump_pub.publish(msg)
        msg.data = '/recording/lidar,event,STOP'
        self._dump_pub.publish(msg)

        self._recording = False

        if self._cam_timer is not None:
            self._cam_timer.cancel()
            self._cam_timer = None
        if self._lidar_timer is not None:
            self._lidar_timer.cancel()
            self._lidar_timer = None

        if self._cam_writer is not None:
            self._cam_writer.release()
            self._cam_writer = None
        if self._lidar_writer is not None:
            self._lidar_writer.release()
            self._lidar_writer = None

        self.get_logger().info('Recording stopped — videos saved')

    # -- Frame writers ---------------------------------------------------------

    def _write_camera_frame(self):
        if self._latest_img_msg is None or self._cam_writer is None:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(self._latest_img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return
        h, w = img.shape[:2]
        # Center-crop to 16:9 aspect ratio
        target_ratio = CAM_WIDTH / CAM_HEIGHT  # 16:9
        src_ratio = w / h
        if src_ratio > target_ratio:
            # Source is wider — crop width
            new_w = int(h * target_ratio)
            x0 = (w - new_w) // 2
            crop = img[:, x0:x0 + new_w]
        else:
            # Source is taller — crop height
            new_h = int(w / target_ratio)
            y0 = (h - new_h) // 2
            crop = img[y0:y0 + new_h, :]
        thumb = cv2.resize(crop, (CAM_WIDTH, CAM_HEIGHT), interpolation=cv2.INTER_AREA)
        self._cam_writer.write(thumb)

    def _write_lidar_frame(self):
        if self._latest_scan is None or self._lidar_writer is None:
            return
        scan = self._latest_scan
        canvas = np.zeros((LIDAR_SIZE, LIDAR_SIZE, 3), dtype=np.uint8)

        max_range = scan.range_max if scan.range_max > 0 else 10.0
        scale = (LIDAR_SIZE // 2 - 5) / max_range
        cx, cy = LIDAR_SIZE // 2, LIDAR_SIZE // 2

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                angle += scan.angle_increment
                continue

            # Hit point in pixel coords (x forward → up on canvas)
            # LiDAR is mounted upside-down, so negate angle to correct
            hx = int(cx + r * math.sin(angle) * scale)
            hy = int(cy + r * math.cos(angle) * scale)

            # Shadow line from hit outward to max range
            sx = int(cx + max_range * math.sin(angle) * scale)
            sy = int(cy + max_range * math.cos(angle) * scale)
            cv2.line(canvas, (hx, hy), (sx, sy), (40, 40, 40), 1)

            # Green hit dot
            cv2.circle(canvas, (hx, hy), 1, (0, 255, 0), -1)

            angle += scan.angle_increment

        # Red robot origin
        cv2.circle(canvas, (cx, cy), 3, (0, 0, 255), -1)

        self._lidar_writer.write(canvas)


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Release writers on shutdown
        if node._cam_writer is not None:
            node._cam_writer.release()
        if node._lidar_writer is not None:
            node._lidar_writer.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
