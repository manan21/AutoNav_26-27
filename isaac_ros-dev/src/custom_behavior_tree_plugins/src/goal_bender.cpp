#include "gradient_escape/goal_bender.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

#include "breadcrumb/forward_blocked_check.hpp"

namespace gradient_escape
{

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

  // Track breadcrumb-buffer emptiness so the bend logic can fire even
  // when the goal is in front, provided breadcrumb-reverse has nothing
  // left to spend. Latched / transient-local so we pick up the most
  // recent buffer state on subscription.
  breadcrumb_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/breadcrumb_tail",
    rclcpp::QoS(1).transient_local(),
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      breadcrumb_buffer_empty_.store(msg->poses.empty());
    });
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
  double min_lookahead_m = 0.6;
  getInput("bend_distance", bend_dist);
  getInput("angle_threshold", angle_thresh);
  getInput("bend_angle", bend_angle);
  getInput("path_lookahead_index", path_lookahead_idx);
  getInput("min_lookahead_m", min_lookahead_m);

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

  nav_msgs::msg::Path prev_path;
  getInput("previous_path", prev_path);
  // Pure-pursuit lookahead (sim-validated) when min_lookahead_m > 0,
  // else legacy fixed-index.
  const auto fb = (min_lookahead_m > 0.0)
    ? breadcrumb::computeForwardBlockedLookahead(
        rx, ry, ryaw, gx, gy, prev_path, angle_thresh, min_lookahead_m)
    : breadcrumb::computeForwardBlocked(
        rx, ry, ryaw, gx, gy, prev_path, angle_thresh, path_lookahead_idx);

  // Bend trigger:
  //   * goal AND path both behind                  → forward-bend (always)
  //   * path behind, goal in front, no breadcrumbs → forward-bend (turn around)
  //   * path in front (or no path yet)             → pass through
  //   * path behind, goal in front, breadcrumbs    → pass through; the
  //                                                  breadcrumb-reverse
  //                                                  behavior handles this.
  const bool buffer_empty = breadcrumb_buffer_empty_.load();
  const bool bend_now =
    fb.path_valid &&
    fb.path_behind &&
    (fb.goal_behind || buffer_empty);

  if (!bend_now) {
    setOutput("output_goal", goal);
    if (nav_goal_pub_) nav_goal_pub_->publish(goal);
    return BT::NodeStatus::SUCCESS;
  }

  // Forward-bend: place an intermediate goal bend_distance metres away,
  // offset by bend_angle from the robot heading toward the side the real
  // goal is on. Yields a forward-only plan that turns toward the real
  // goal incrementally — exactly what DWB in forward-only mode
  // (min_vel_x: 0.0) can follow.
  const double offset = (fb.rel_goal >= 0.0) ? bend_angle : -bend_angle;
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
    "GoalBender: bending -> (%.1f, %.1f) [goal_behind=%d path_behind=%d crumbs_empty=%d]",
    bent.pose.position.x, bent.pose.position.y,
    static_cast<int>(fb.goal_behind),
    static_cast<int>(fb.path_behind),
    static_cast<int>(buffer_empty));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace gradient_escape

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gradient_escape::GoalBender>("GoalBender");
}
