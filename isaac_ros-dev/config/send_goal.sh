#!/bin/bash
# Send a Nav2 goal pose from the command line.
# Usage: ./send_goal.sh [-r] <x> <y> [yaw_degrees]
#   -r            relative mode: x/y/yaw are relative to the robot's current pose
#   yaw_degrees   defaults to 0 (facing +x)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

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

# Publish the same /goal_pose message path RViz uses. In this stack,
# map_padder also consumes /goal_pose to grow /map_padded around the
# destination; sending only a NavigateToPose action can be accepted by
# Nav2 while the global costmap never expands toward the goal.
python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw_deg = float(sys.argv[3])

GOAL_TOPIC = '/goal_pose'
WAIT_FOR_SUB_TIMEOUT_S = 5.0
POST_PUBLISH_SPIN_S = 0.5


def _subscriber_names(node):
    try:
        infos = node.get_subscriptions_info_by_topic(GOAL_TOPIC)
    except Exception:
        return []

    names = []
    for info in infos:
        namespace = getattr(info, 'node_namespace', '') or ''
        name = getattr(info, 'node_name', '') or ''
        if not name:
            continue
        if namespace and namespace != '/':
            names.append(f'{namespace.rstrip("/")}/{name}')
        else:
            names.append(f'/{name}')
    return sorted(set(names))


def _has_node(names, node_name):
    return any(name == f'/{node_name}' or name.endswith(f'/{node_name}')
               for name in names)

rclpy.init()
node = Node('send_goal_pose_pub')
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
)
pub = node.create_publisher(PoseStamped, GOAL_TOPIC, qos)

start = time.monotonic()
deadline = start + WAIT_FOR_SUB_TIMEOUT_S
names = []
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    names = _subscriber_names(node)
    if _has_node(names, 'bt_navigator') and _has_node(names, 'map_padder'):
        break
    if not names and pub.get_subscription_count() >= 2 and time.monotonic() - start > 0.5:
        break

sub_count = pub.get_subscription_count()
if sub_count < 1:
    print(f'WARNING: no subscribers matched {GOAL_TOPIC} after '
          f'{WAIT_FOR_SUB_TIMEOUT_S}s; publishing anyway',
          file=sys.stderr)
elif not _has_node(names, 'bt_navigator') or not _has_node(names, 'map_padder'):
    seen = ', '.join(names) if names else f'{sub_count} anonymous subscriber(s)'
    print(f'WARNING: expected bt_navigator and map_padder on {GOAL_TOPIC}; '
          f'saw {seen}. Publishing anyway.',
          file=sys.stderr)

msg = PoseStamped()
msg.header.frame_id = 'map'
msg.pose.position.x = x
msg.pose.position.y = y
msg.pose.position.z = 0.0
msg.pose.orientation.x = 0.0
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
msg.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)

msg.header.stamp = node.get_clock().now().to_msg()
pub.publish(msg)

end = time.monotonic() + POST_PUBLISH_SPIN_S
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)

print(f'Published {GOAL_TOPIC}; Nav2 should start as it does from RViz.')

node.destroy_node()
rclpy.shutdown()
PYEOF
