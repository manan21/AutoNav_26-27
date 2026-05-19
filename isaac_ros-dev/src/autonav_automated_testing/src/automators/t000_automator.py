#!/usr/bin/env python3
"""
t000_automator.py - DAQ Mode Test Automator

Objective:
- Provide a minimal, operator-driven data acquisition test.
- Start/Stop data collection via joystick A button toggle.

Behavior:
- Do no driving automatically, collect data while operator drives robot.
"""

import sys

import rclpy
from base_automator import BaseAutomator
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, Joy, LaserScan, NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class T000Automator(BaseAutomator):
    def __init__(self):
        super().__init__('t000_automater', 't000', 'DAQ_MODE')

        # State — all false by default, updated when messages arrive
        self.odom_online = False
        self.joy_online = False
        self.gps_online = False
        self.imu_online = False
        self._cam_rec_online = False
        self._lidar_rec_online = False
        self.A_BUTTON_INDEX = 0
        self.last_joy_buttons = None

        # Publishers
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps_fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)
        self.create_subscription(
            Image, '/zed/zed_node/rgb/color/rect/image',
            self._cam_rec_cb, SENSOR_QOS,
        )
        self.create_subscription(
            LaserScan, '/scan_fullframe',
            self._lidar_rec_cb, SENSOR_QOS,
        )

        # Status display timer — prints a visible status box every 5 seconds
        self.status_timer = self.create_timer(5.0, self.print_status)

        self.get_logger().info('T000 Automator initialized — press A to start DAQ')

    # ===== Status Display ===== #
    def _tag(self, online):
        return 'ONLINE' if online else '------'

    def _daq_state(self):
        if self.test_started and not self.test_complete:
            return '** RECORDING **'
        elif self.test_complete:
            return 'COMPLETE'
        return 'IDLE (Press A)'

    def print_status(self):
        self.get_logger().info(
            '\n'
            '######################################\n'
            '#         T000 DAQ MODE              #\n'
            '######################################\n'
            '#  Odom : %-8s  GPS : %-8s #\n'
            '#  Joy  : %-8s  IMU : %-8s #\n'
            '#  CamRec : %-6s  LidRec : %-6s #\n'
            '#  DAQ  : %-28s #\n'
            '#  A = Start/Stop  Ctrl-C = Save     #\n'
            '######################################'
            % (
                self._tag(self.odom_online), self._tag(self.gps_online),
                self._tag(self.joy_online), self._tag(self.imu_online),
                self._tag(self._cam_rec_online), self._tag(self._lidar_rec_online),
                self._daq_state()
            )
        )

    # ===== Callbacks ===== #
    def odom_callback(self, msg: Odometry):
        if not self.odom_online:
            self.odom_online = True
            self.get_logger().info('Odometry online')

    def gps_callback(self, msg):
        if not self.gps_online:
            self.gps_online = True
            self.get_logger().info('GPS online')

    def imu_callback(self, msg):
        if not self.imu_online:
            self.imu_online = True
            self.get_logger().info('IMU online')

    def _cam_rec_cb(self, msg):
        if not self._cam_rec_online:
            self._cam_rec_online = True
            self.get_logger().info('Camera record online')

    def _lidar_rec_cb(self, msg):
        if not self._lidar_rec_online:
            self._lidar_rec_online = True
            self.get_logger().info('LiDAR record online')

    def joy_callback(self, msg: Joy):
        if not self.joy_online:
            self.joy_online = True
            self.get_logger().info('Joystick online')
        if not hasattr(msg, 'buttons') or len(msg.buttons) <= self.A_BUTTON_INDEX:
            self.last_joy_buttons = list(msg.buttons) if hasattr(msg, 'buttons') else None
            return
        curr = list(msg.buttons)
        if self.last_joy_buttons is not None:
            prev = self.last_joy_buttons
            prev_val = prev[self.A_BUTTON_INDEX] if len(prev) > self.A_BUTTON_INDEX else 0
            curr_val = curr[self.A_BUTTON_INDEX]
            if curr_val == 1 and prev_val == 0:
                # Rising edge on A button — no lockout, always allow start/stop
                if not self.test_started and not self.test_complete:
                    self.get_logger().info('A pressed — starting DAQ mode')
                    try:
                        self.start_test()
                    except Exception as e:
                        self.get_logger().warn(f'Failed to start DAQ: {e}')
                elif self.test_started and not self.test_complete:
                    self.get_logger().info('A pressed again — stopping DAQ mode')
                    try:
                        self.stop_test()
                    except Exception as e:
                        self.get_logger().warn(f'Failed to stop DAQ: {e}')
        self.last_joy_buttons = curr

    # ===== Test Lifecycle ===== #
    def test_actions(self):
        # No autonomous actions; data collection only under operator control
        self.get_logger().info('DAQ Mode active — operator controls everything. Press A again to stop.')


def main(args=None):
    rclpy.init(args=args)
    automator = None
    try:
        automator = T000Automator()
        rclpy.spin(automator)
    except KeyboardInterrupt:
        print('\n[INFO] Keyboard interrupt detected (Ctrl+C)')
        if automator is not None:
            # Stop the bag recorder FIRST — without this, SIGINT to
            # the automator orphans the bag (it's in its own process
            # group via os.setsid, so it doesn't catch our SIGINT
            # propagation).
            try:
                automator._stop_bag_record()
            except Exception as e:
                print(f'[WARN] Failed to stop bag recorder: {e}')
            if automator.enable_legacy_capture:
                print('[INFO] Saving collected data before shutdown...')
                try:
                    automator.save_data()
                    print(f'[INFO] Data saved to: {automator.log_file}')
                except Exception as e:
                    print(f'[ERROR] Failed to save data: {e}')
    except Exception as e:
        print(f'[ERROR] Unexpected error: {e}')
        if automator is not None:
            try:
                automator._stop_bag_record()
            except Exception:
                pass
            if automator.enable_legacy_capture:
                try:
                    automator.save_data()
                    print(f'[INFO] Data saved to: {automator.log_file}')
                except:
                    pass
    finally:
        if automator is not None:
            try:
                automator.destroy_node()
            except:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
