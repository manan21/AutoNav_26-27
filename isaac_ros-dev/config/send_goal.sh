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

# Publish to /goal_pose with a long-lived publisher. Both bt_navigator
# (which translates the topic into a NavigateToPose action) and
# map_padder (which extends the global-costmap corridor toward the goal)
# subscribe here — if map_padder misses the message, the goal lands in a
# LETHAL region and the planner can't path into it. We therefore wait
# for BOTH expected subscribers before the first publish, then re-spam
# the goal at 1 Hz until /odom shows the robot has actually started
# moving (auto mode kicks the planner off). Without this, a single
# publish that arrives during Nav2's "Activating planner_server" window
# is silently dropped and the operator sees no motion — by the time
# they notice and re-issue, autonomous-mode state may have moved on.
python3 - "$X" "$Y" "$YAW_DEG" <<'PYEOF'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw_deg = float(sys.argv[3])

# bt_navigator + map_padder. Raise if more /goal_pose subscribers are
# added; partial delivery presents as silent path-planning failure.
EXPECTED_SUB_COUNT = 2
WAIT_FOR_SUB_TIMEOUT_S = 10.0
SPAM_HZ = 1.0
SPAM_TIMEOUT_S = 60.0
# Motion thresholds for the exit condition — chosen above /odom noise
# floor on the AutoNav drivetrain (encoder ticks at rest jitter well
# under 0.02 m/s and 0.02 rad/s).
MOVING_LIN_THRESH = 0.05  # m/s
MOVING_ANG_THRESH = 0.05  # rad/s
# Require the robot to be moving for two consecutive checks (~1 s at
# 1 Hz spam rate) so a single noisy velocity sample doesn't end the
# spam early before Nav2 has actually committed to a path.
MOVING_CONSEC_REQUIRED = 2

rclpy.init()
node = Node('send_goal_pose_pub')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
pub = node.create_publisher(PoseStamped, '/goal_pose', qos)

# Latest twist from /odom, updated by the subscription callback.
state = {'lin': 0.0, 'ang': 0.0, 'have': False}
def _odom_cb(msg):
    lx = msg.twist.twist.linear.x
    ly = msg.twist.twist.linear.y
    az = msg.twist.twist.angular.z
    state['lin'] = math.hypot(lx, ly)
    state['ang'] = abs(az)
    state['have'] = True
odom_sub = node.create_subscription(Odometry, '/odom', _odom_cb, 10)

# Build the goal message once — header.stamp is refreshed on each spam.
def _make_msg():
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.0
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
    m.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
    return m

# Wait for subscribers on the FIRST publish so the initial drop doesn't
# race Nav2 lifecycle activation. Re-spams below skip the wait — by
# then either Nav2 is up and listening, or it never will be.
deadline = time.monotonic() + WAIT_FOR_SUB_TIMEOUT_S
while time.monotonic() < deadline and pub.get_subscription_count() < EXPECTED_SUB_COUNT:
    rclpy.spin_once(node, timeout_sec=0.1)
matched = pub.get_subscription_count()
if matched < EXPECTED_SUB_COUNT:
    print(f'WARNING: only {matched}/{EXPECTED_SUB_COUNT} subscribers on '
          f'/goal_pose after {WAIT_FOR_SUB_TIMEOUT_S}s — publishing anyway '
          f'(delivery may be partial; planner may not path)',
          file=sys.stderr)

# Spam loop. Publish once, then on each tick: drain /odom for ~1/SPAM_HZ
# seconds and decide whether to republish. If motion is detected, we
# STOP publishing and just keep spinning until the debounce window
# (MOVING_CONSEC_REQUIRED consecutive moving samples) confirms a real
# path execution — that way the first publish that actually triggers
# motion is the LAST publish. Hard cap at SPAM_TIMEOUT_S so we never
# wedge if auto mode is off or Nav2 silently refuses the goal.
interval = 1.0 / SPAM_HZ
start = time.monotonic()
publish_count = 0
moving_consec = 0
done_reason = 'timeout'

pub.publish(_make_msg())
publish_count += 1

while time.monotonic() - start < SPAM_TIMEOUT_S:
    tick_end = time.monotonic() + interval
    while time.monotonic() < tick_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if state['have'] and (state['lin'] > MOVING_LIN_THRESH
                          or state['ang'] > MOVING_ANG_THRESH):
        moving_consec += 1
        if moving_consec >= MOVING_CONSEC_REQUIRED:
            done_reason = 'moving'
            break
        # Robot is moving but the debounce isn't satisfied yet — just
        # spin another interval to confirm. Do NOT republish; we want
        # the moving robot to keep executing the goal Nav2 already
        # accepted, not be retriggered.
        continue
    # Not moving: reset the debounce and try again.
    moving_consec = 0
    pub.publish(_make_msg())
    publish_count += 1

elapsed = time.monotonic() - start
if done_reason == 'moving':
    print(f'Robot moving after {publish_count} publish(es), '
          f'{elapsed:.1f}s — exiting.')
else:
    print(f'WARNING: robot did not start moving after {publish_count} '
          f'publish(es) over {elapsed:.1f}s. Check that auto mode is on '
          f'and Nav2 is active.', file=sys.stderr)

node.destroy_subscription(odom_sub)
node.destroy_node()
rclpy.shutdown()
PYEOF
