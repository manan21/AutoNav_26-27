#!/bin/bash
# Send a Nav2 goal pose from the command line.
# Usage: ./send_goal.sh [-r] <x> <y> [yaw_degrees]
#   -r             relative mode: x/y are desired nav_center travel in meters,
#                  yaw is relative to the current nav_center heading
#   yaw_degrees    defaults to 0 (facing +x)

RELATIVE=false

usage() {
  echo "Usage: $0 [-r] <x> <y> [yaw_degrees]"
  echo "  -r             goal is relative to the robot's current pose"
}

while [ $# -gt 0 ]; do
  case "$1" in
    -r|--relative)
      RELATIVE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
    *)
      break
      ;;
  esac
done

if [ $# -lt 2 ]; then
  usage
  exit 1
fi

X=$1
Y=$2
YAW_DEG=${3:-0}

if $RELATIVE; then
  # Look up map->nav_center directly via tf2_ros Python API.
  # Retries until a valid transform is available or timeout elapses —
  # avoids the "bad transform" text that tf2_echo prints during startup.
  GLOBAL=$(python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

dx = float(sys.argv[1])
dy = float(sys.argv[2])
dyaw_deg = float(sys.argv[3])

TARGET_FRAME = 'map'
SOURCE_FRAME = os.environ.get('SEND_GOAL_RELATIVE_FRAME', 'nav_center')
XY_GOAL_TOLERANCE_M = float(os.environ.get('SEND_GOAL_XY_GOAL_TOLERANCE_M', '0.25'))
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
requested_dist = math.hypot(dx, dy)

# In relative mode, x/y are operator-facing travel commands. Nav2 declares
# success when robot_base_frame is within xy_goal_tolerance of the target, so
# place the goal one tolerance farther along the requested translation vector.
# Pure yaw goals keep the same position.
if requested_dist > 1e-6:
    target_dist = requested_dist + XY_GOAL_TOLERANCE_M
    scale = target_dist / requested_dist
    goal_dx = dx * scale
    goal_dy = dy * scale
else:
    target_dist = requested_dist
    goal_dx = dx
    goal_dy = dy

goal_x = rx + goal_dx * math.cos(ryaw) - goal_dy * math.sin(ryaw)
goal_y = ry + goal_dx * math.sin(ryaw) + goal_dy * math.cos(ryaw)
goal_yaw_deg = math.degrees(ryaw + dyaw)

print(
    f'Relative goal from {SOURCE_FRAME}: requested={requested_dist:.3f}m, '
    f'goal_offset={target_dist:.3f}m, xy_tolerance={XY_GOAL_TOLERANCE_M:.3f}m',
    file=sys.stderr,
)
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

# Publish to /goal_pose with a long-lived publisher. Both bt_navigator
# (which translates the topic into a NavigateToPose action) and
# map_padder (which extends the global-costmap corridor toward the goal)
# subscribe here — if map_padder misses the message, the goal lands in a
# LETHAL region and the planner can't path into it. We therefore wait
# for BOTH expected subscribers, then hold the publisher alive long
# enough for RELIABLE delivery to ACK before tearing down.
python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw_deg = float(sys.argv[3])

# bt_navigator + map_padder. Raise if more /goal_pose subscribers are
# added; partial delivery presents as silent path-planning failure.
EXPECTED_SUB_COUNT = 2
WAIT_FOR_SUB_TIMEOUT_S = 10.0
POST_PUBLISH_SPIN_S = 3.0

rclpy.init()
node = Node('send_goal_pose_pub')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
pub = node.create_publisher(PoseStamped, '/goal_pose', qos)

deadline = time.monotonic() + WAIT_FOR_SUB_TIMEOUT_S
while time.monotonic() < deadline and pub.get_subscription_count() < EXPECTED_SUB_COUNT:
    rclpy.spin_once(node, timeout_sec=0.1)

matched = pub.get_subscription_count()
if matched < EXPECTED_SUB_COUNT:
    print(f'WARNING: only {matched}/{EXPECTED_SUB_COUNT} subscribers on '
          f'/goal_pose after {WAIT_FOR_SUB_TIMEOUT_S}s — publishing anyway '
          f'(delivery may be partial; planner may not path)',
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
