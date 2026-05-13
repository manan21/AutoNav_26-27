#!/bin/bash
# Send a Nav2 goal pose from the command line.
# Usage: ./send_goal.sh [-r] <x> <y> [yaw_degrees]
#   -r            relative mode: x/y/yaw are relative to the robot's current pose
#   yaw_degrees   defaults to 0 (facing +x)

RELATIVE=false
if [ "$1" = "-r" ]; then
  RELATIVE=true
  shift
fi

if [ $# -lt 2 ]; then
  echo "Usage: $0 [-r] <x> <y> [yaw_degrees]"
  echo "  -r  goal is relative to the robot's current pose"
  exit 1
fi

X=$1
Y=$2
YAW_DEG=${3:-0}

if $RELATIVE; then
  # Look up map->base_link directly via tf2_ros Python API.
  # Retries until a valid transform is available or timeout elapses —
  # avoids the "bad transform" text that tf2_echo prints during startup.
  GLOBAL=$(python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

dx = float(sys.argv[1])
dy = float(sys.argv[2])
dyaw_deg = float(sys.argv[3])

TARGET_FRAME = 'map'
SOURCE_FRAME = 'base_link'
TIMEOUT_SEC = 10.0
POLL_HZ = 10.0

rclpy.init()
node = Node('send_goal_tf_lookup')
buf = Buffer()
TransformListener(buf, node, spin_thread=False)

deadline = time.monotonic() + TIMEOUT_SEC
tf = None
last_err = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=1.0 / POLL_HZ)
    try:
        tf = buf.lookup_transform(TARGET_FRAME, SOURCE_FRAME, rclpy.time.Time(),
                                  timeout=Duration(seconds=0.0))
        break
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        last_err = e
        continue

node.destroy_node()
rclpy.shutdown()

if tf is None:
    print(f'ERROR: no {TARGET_FRAME}->{SOURCE_FRAME} transform within {TIMEOUT_SEC}s ({last_err})',
          file=sys.stderr)
    sys.exit(1)

rx = tf.transform.translation.x
ry = tf.transform.translation.y
qz = tf.transform.rotation.z
qw = tf.transform.rotation.w
ryaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

dyaw = math.radians(dyaw_deg)
goal_x = rx + dx * math.cos(ryaw) - dy * math.sin(ryaw)
goal_y = ry + dx * math.sin(ryaw) + dy * math.cos(ryaw)
goal_yaw_deg = math.degrees(ryaw + dyaw)

print(f'{goal_x} {goal_y} {goal_yaw_deg}')
PYEOF
)

  if [ $? -ne 0 ] || [ -z "$GLOBAL" ]; then
    echo "Failed to get current pose. Is the robot localized and /tf publishing?"
    exit 1
  fi

  read X Y YAW_DEG <<< "$GLOBAL"
  echo "Relative goal resolved to global: x=$X, y=$Y, yaw=${YAW_DEG}°"
fi

echo "Sending goal: x=$X, y=$Y, yaw=${YAW_DEG}°"

# Send via the NavigateToPose action client — the same path RViz's
# "Nav2 Goal" tool uses. Bypasses the /goal_pose topic, which means no
# DDS-discovery race between an ephemeral publisher and bt_navigator's
# subscriber, and preempts any in-flight action goal cleanly.
python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw_deg = float(sys.argv[3])

ACTION_NAME = '/navigate_to_pose'
SERVER_WAIT_TIMEOUT_S = 5.0
GOAL_ACCEPT_TIMEOUT_S = 5.0

rclpy.init()
node = Node('send_goal_action_client')
client = ActionClient(node, NavigateToPose, ACTION_NAME)

if not client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_S):
    print(f'ERROR: {ACTION_NAME} action server not available after '
          f'{SERVER_WAIT_TIMEOUT_S}s (is bt_navigator running?)',
          file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
goal_msg.pose.pose.position.x = x
goal_msg.pose.pose.position.y = y
goal_msg.pose.pose.position.z = 0.0
goal_msg.pose.pose.orientation.x = 0.0
goal_msg.pose.pose.orientation.y = 0.0
goal_msg.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
goal_msg.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)

send_future = client.send_goal_async(goal_msg)
rclpy.spin_until_future_complete(
    node, send_future, timeout_sec=GOAL_ACCEPT_TIMEOUT_S)

goal_handle = send_future.result()
if goal_handle is None:
    print(f'ERROR: send_goal_async did not complete within '
          f'{GOAL_ACCEPT_TIMEOUT_S}s', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

if not goal_handle.accepted:
    print('ERROR: NavigateToPose goal REJECTED by server', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

print('NavigateToPose goal accepted — robot is rerouting.')

node.destroy_node()
rclpy.shutdown()
PYEOF
