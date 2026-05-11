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

# Hold the publisher alive until a subscriber matches and the message
# is flushed. `ros2 topic pub --once` drops the message on the floor if
# NAV2's /goal_pose subscriber hasn't completed DDS discovery against
# our ephemeral publisher yet — this is what was breaking send_goal.sh
# while the GPS handler (long-lived publisher) kept working.
python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw_deg = float(sys.argv[3])

WAIT_FOR_SUB_TIMEOUT_S = 5.0
POST_PUBLISH_SPIN_S = 0.5

rclpy.init()
node = Node('send_goal_pose_pub')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
pub = node.create_publisher(PoseStamped, '/goal_pose', qos)

deadline = time.monotonic() + WAIT_FOR_SUB_TIMEOUT_S
while time.monotonic() < deadline and pub.get_subscription_count() < 1:
    rclpy.spin_once(node, timeout_sec=0.1)

if pub.get_subscription_count() < 1:
    print(f'WARNING: no subscriber on /goal_pose after {WAIT_FOR_SUB_TIMEOUT_S}s '
          f'(is bt_navigator running?) — publishing anyway',
          file=sys.stderr)

msg = PoseStamped()
msg.header.frame_id = 'map'
msg.header.stamp = node.get_clock().now().to_msg()
msg.pose.position.x = x
msg.pose.position.y = y
msg.pose.position.z = 0.0
msg.pose.orientation.x = 0.0
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
msg.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
pub.publish(msg)

end = time.monotonic() + POST_PUBLISH_SPIN_S
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)

node.destroy_node()
rclpy.shutdown()
PYEOF
