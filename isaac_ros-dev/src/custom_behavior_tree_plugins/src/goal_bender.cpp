#include "gradient_escape/goal_bender.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace gradient_escape
{

namespace
{
// Normalize an angle to [-pi, pi].
inline double wrap_pi(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

GoalBender::GoalBender(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GoalBender::tick()
{
  // --- read ports ---
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("input_goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }

  double bend_dist = 1.5;
  double angle_thresh = 1.57;   // ~90 deg
  double bend_angle = 1.05;     // ~60 deg
  int    path_lookahead_idx = 5;
  getInput("bend_distance", bend_dist);
  getInput("angle_threshold", angle_thresh);
  getInput("bend_angle", bend_angle);
  getInput("path_lookahead_index", path_lookahead_idx);

  // --- get robot pose from blackboard TF ---
  auto tf_buffer =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Can't get pose — pass goal through unchanged
    setOutput("output_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double ryaw = tf2::getYaw(tf.transform.rotation);

  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  const double rel_goal = wrap_pi(std::atan2(gy - ry, gx - rx) - ryaw);
  const bool goal_behind = std::abs(rel_goal) > angle_thresh;

  nav_msgs::msg::Path prev_path;
  const bool have_path =
    getInput("previous_path", prev_path) && prev_path.poses.size() >= 2;
  bool path_behind = false;
  if (have_path) {
    const int idx = std::min(
      static_cast<int>(prev_path.poses.size()) - 1,
      std::max(1, path_lookahead_idx));
    const double lx = prev_path.poses[idx].pose.position.x;
    const double ly = prev_path.poses[idx].pose.position.y;
    const double rel_look =
      wrap_pi(std::atan2(ly - ry, lx - rx) - ryaw);
    path_behind = std::abs(rel_look) > angle_thresh;
  }

  if (!goal_behind && !path_behind) {
    setOutput("output_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped bent;
  bent.header = goal.header;
  bent.header.stamp = node->get_clock()->now();
  bent.pose.position.z = 0.0;
  bool used_path_waypoint = false;

  if (have_path) {
    int last_forward = -1;
    for (size_t i = 0; i < prev_path.poses.size(); ++i) {
      const double px = prev_path.poses[i].pose.position.x;
      const double py = prev_path.poses[i].pose.position.y;
      const double dx = px - rx;
      const double dy = py - ry;
      const double d  = std::hypot(dx, dy);
      if (d < 0.3) {  // skip waypoints inside footprint — angle ill-defined
        continue;
      }
      const double rel_p = wrap_pi(std::atan2(dy, dx) - ryaw);
      if (std::abs(rel_p) > angle_thresh) {
        break;
      }
      last_forward = static_cast<int>(i);
    }
    if (last_forward >= 0) {
      bent.pose.position.x = prev_path.poses[last_forward].pose.position.x;
      bent.pose.position.y = prev_path.poses[last_forward].pose.position.y;
      used_path_waypoint = true;
    }
  }

  if (!used_path_waypoint) {
    const double offset = (rel_goal >= 0.0) ? bend_angle : -bend_angle;
    const double heading = ryaw + offset;
    bent.pose.position.x = rx + bend_dist * std::cos(heading);
    bent.pose.position.y = ry + bend_dist * std::sin(heading);
  }

  const double to_real = std::atan2(
    gy - bent.pose.position.y, gx - bent.pose.position.x);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, to_real);
  bent.pose.orientation = tf2::toMsg(q);

  setOutput("output_goal", bent);

  RCLCPP_INFO(node->get_logger(),
    "GoalBender: trigger=%s%s%s -> intermediate (%.1f, %.1f) via %s",
    goal_behind ? "goal-behind" : "",
    (goal_behind && path_behind) ? "+" : "",
    path_behind ? "path-behind" : "",
    bent.pose.position.x, bent.pose.position.y,
    used_path_waypoint ? "path-walk" : "fixed-arc");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace gradient_escape

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gradient_escape::GoalBender>("GoalBender");
}
