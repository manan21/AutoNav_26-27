#include "gradient_escape/goal_bender.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace gradient_escape
{

namespace
{
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
  // Publish the chosen goal each tick so map_padder can pad the
  // corridor toward the actual planner target. RELIABLE QoS so map_padder
  // doesn't miss the first tick when the corridor most needs to extend.
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  nav_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/nav_goal", rclcpp::QoS(1).reliable());
}

BT::NodeStatus GoalBender::tick()
{
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

  auto tf_buffer =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    setOutput("output_goal", goal);
    if (nav_goal_pub_) nav_goal_pub_->publish(goal);
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

  // Bend ONLY when both the real goal AND the previous path are
  // behind the robot. Other cases:
  //   - goal in front, path in front  -> pass through (normal nav)
  //   - goal in front, path behind    -> pass through; the backup-
  //                                       recovery BT node handles
  //                                       breadcrumb backtracking,
  //                                       not us.
  //   - goal behind, path in front    -> pass through; planner has
  //                                       already found a forward
  //                                       route.
  if (!(goal_behind && path_behind)) {
    setOutput("output_goal", goal);
    if (nav_goal_pub_) nav_goal_pub_->publish(goal);
    return BT::NodeStatus::SUCCESS;
  }

  // Forward-bend: place an intermediate goal bend_distance metres
  // away, offset by bend_angle from the robot heading toward the
  // side the real goal is on. Yields a forward-only plan that turns
  // toward the real goal incrementally — exactly what DWB in
  // forward-only mode (min_vel_x: 0.0) can follow.
  const double offset = (rel_goal >= 0.0) ? bend_angle : -bend_angle;
  const double heading = ryaw + offset;

  geometry_msgs::msg::PoseStamped bent;
  bent.header = goal.header;
  bent.header.stamp = node->get_clock()->now();
  bent.pose.position.x = rx + bend_dist * std::cos(heading);
  bent.pose.position.y = ry + bend_dist * std::sin(heading);
  bent.pose.position.z = 0.0;

  const double to_real = std::atan2(
    gy - bent.pose.position.y, gx - bent.pose.position.x);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, to_real);
  bent.pose.orientation = tf2::toMsg(q);

  setOutput("output_goal", bent);
  if (nav_goal_pub_) nav_goal_pub_->publish(bent);

  RCLCPP_INFO(node->get_logger(),
    "GoalBender: goal+path both behind -> forward bend to (%.1f, %.1f)",
    bent.pose.position.x, bent.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace gradient_escape

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gradient_escape::GoalBender>("GoalBender");
}
