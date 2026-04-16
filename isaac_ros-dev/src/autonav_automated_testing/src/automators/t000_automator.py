#!/usr/bin/env python3
"""
t000_automator.py - DAQ Mode Test Automator

Objective:
- Provide a minimal, operator-driven data acquisition test.
- Start/Stop data collection via joystick A button toggle.

Behavior:
- Do no driving automatically, collect data while operator drives robot.
"""

import os
import sys

# Force unbuffered stdout so ANSI escape codes flush immediately under ROS2 launch
if not os.environ.get('PYTHONUNBUFFERED'):
    os.environ['PYTHONUNBUFFERED'] = '1'

import rclpy
from base_automator import BaseAutomator
from sensor_msgs.msg import Joy, NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# Number of lines reserved for the persistent status header
STATUS_LINES = 9

def setup_scroll_region():
    """Set terminal scroll region below the status header."""
    sys.stdout.write(f'\033[{STATUS_LINES + 1};r')
    sys.stdout.write(f'\033[{STATUS_LINES + 1};1H')
    sys.stdout.flush()

def draw_status(odom, joy, gps, imu, collecting, test_started):
    """Redraw the persistent status box at the top of the terminal."""
    def tag(online):
        return '\033[32mONLINE \033[0m' if online else '\033[31mOFFLINE\033[0m'

    if collecting:
        mode = '\033[32m RECORDING \033[0m'
    elif test_started:
        mode = '\033[33m PAUSED \033[0m'
    else:
        mode = '\033[36m READY (Press A) \033[0m'

    lines = [
        '\033[44;97m            T000 DAQ MODE             \033[0m',
        f'  Odometry : {tag(odom)}    GPS : {tag(gps)}',
        f'  Joystick : {tag(joy)}    IMU : {tag(imu)}',
        f'  DAQ      : {mode}',
        '\033[44;97m                                      \033[0m',
        '  A = Start/Stop   Ctrl-C = Save+Quit',
        '\033[44;97m                                      \033[0m',
    ]

    sys.stdout.write('\033[s')  # save cursor position
    for i, line in enumerate(lines):
        sys.stdout.write(f'\033[{i + 1};1H\033[2K{line}')
    sys.stdout.write('\033[u')  # restore cursor position
    sys.stdout.flush()


class T000Automator(BaseAutomator):
    def __init__(self):
        super().__init__('t000_automater', 't000', 'DAQ_MODE')

        # State — all false by default, updated when messages arrive
        self.odom_online = False
        self.joy_online = False
        self.gps_online = False
        self.imu_online = False
        self.A_BUTTON_INDEX = 0
        self.last_joy_buttons = None

        # Publishers
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps_fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)

        # Set up persistent header and status refresh timer
        sys.stdout.write('\033[2J\033[H')  # clear screen
        setup_scroll_region()
        self.refresh_status()
        self.status_timer = self.create_timer(1.0, self.refresh_status)

        self.get_logger().info('T000 Automator initialized — press A to start DAQ')

    # ===== Status Display ===== #
    def refresh_status(self):
        draw_status(
            self.odom_online, self.joy_online, self.gps_online, self.imu_online,
            self.test_started and not self.test_complete,
            self.test_started
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
                automator.save_data()
                print(f'[INFO] Data saved to: {automator.log_file}')
            except:
                pass
    finally:
        # Reset scroll region to full terminal
        sys.stdout.write('\033[r\033[999;1H')
        sys.stdout.flush()
        if automator is not None:
            try:
                automator.destroy_node()
            except:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
