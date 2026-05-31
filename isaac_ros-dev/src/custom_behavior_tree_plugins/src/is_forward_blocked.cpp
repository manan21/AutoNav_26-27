#include "breadcrumb/is_forward_blocked.hpp"

#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "breadcrumb/forward_blocked_check.hpp"

namespace breadcrumb
{

IsForwardBlocked::IsForwardBlocked(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsForwardBlocked::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }

  double angle_thresh = 1.57;
  int path_lookahead_idx = 5;
  double min_lookahead_m = 0.6;
  getInput("angle_threshold", angle_thresh);
  getInput("path_lookahead_index", path_lookahead_idx);
  getInput("min_lookahead_m", min_lookahead_m);

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.size() < 2) {
    // No path yet → not forward-blocked.
    return BT::NodeStatus::FAILURE;
  }

  auto tf_buffer =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(
      "map", "nav_center", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return BT::NodeStatus::FAILURE;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double ryaw = tf2::getYaw(tf.transform.rotation);

  // Pure-pursuit lookahead when min_lookahead_m > 0 (the recommended
  // sim-validated path); else fall back to fixed-index for callers that
  // still want the legacy behaviour.
  const auto fb = (min_lookahead_m > 0.0)
    ? computeForwardBlockedLookahead(
        rx, ry, ryaw,
        goal.pose.position.x, goal.pose.position.y,
        path, angle_thresh, min_lookahead_m)
    : computeForwardBlocked(
        rx, ry, ryaw,
        goal.pose.position.x, goal.pose.position.y,
        path, angle_thresh, path_lookahead_idx);

  // The "forward-blocked" case GoalBender deliberately passes through:
  // goal in front of robot, but path turns away behind. That's our cue
  // to engage breadcrumb-reverse.
  const bool forward_blocked =
    fb.path_valid && fb.path_behind && !fb.goal_behind;

  return forward_blocked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace breadcrumb

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<breadcrumb::IsForwardBlocked>("IsForwardBlocked");
}
