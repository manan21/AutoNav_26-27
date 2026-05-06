#!/usr/bin/env python3
"""
Block until a ROS 2 topic publishes its first message, then exit 0.
Exit 1 on timeout, 2 on bad arguments / import error.

Used by GUI launch scripts (run-lidar.sh, run-zed.sh, ...) and by
slam.launch.py to gate dependent nodes on real upstream data.
"""

import argparse
import importlib
import sys
import threading

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


QOS_PRESETS = {
    'sensor': QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    ),
    'reliable': QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    ),
    'transient_local': QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    ),
}


def import_msg_type(spec):
    pkg, _, name = spec.replace('/', '.').rpartition('.')
    if not pkg.endswith('.msg'):
        pkg = pkg.rsplit('.', 1)[0] + '.msg'
    return getattr(importlib.import_module(pkg), name)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('topic')
    p.add_argument('--type', default='sensor_msgs/msg/LaserScan',
                   help='Message type (e.g. sensor_msgs/msg/LaserScan)')
    p.add_argument('--qos', choices=QOS_PRESETS.keys(), default='sensor')
    p.add_argument('--timeout', type=float, default=60.0,
                   help='Seconds to wait before giving up. <=0 means wait forever.')
    args = p.parse_args()

    try:
        msg_type = import_msg_type(args.type)
    except Exception as e:
        print(f'wait_for_topic: bad --type {args.type!r}: {e}', file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node('wait_for_topic_' + args.topic.strip('/').replace('/', '_'))
    received = threading.Event()

    def cb(_msg):
        received.set()

    node.create_subscription(msg_type, args.topic, cb, QOS_PRESETS[args.qos])

    timeout = args.timeout if args.timeout > 0 else None
    deadline_reached = False
    try:
        end = None
        if timeout is not None:
            end = node.get_clock().now().nanoseconds / 1e9 + timeout
        while rclpy.ok() and not received.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if end is not None and node.get_clock().now().nanoseconds / 1e9 >= end:
                deadline_reached = True
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if received.is_set():
        return 0
    if deadline_reached:
        print(f'wait_for_topic: timed out after {args.timeout:.1f}s waiting for {args.topic}',
              file=sys.stderr)
        return 1
    return 1


if __name__ == '__main__':
    sys.exit(main())
