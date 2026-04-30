#include "gradient_escape/goal_bender.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace gradient_escape
{

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
  getInput("bend_distance", bend_dist);
  getInput("angle_threshold", angle_thresh);
  getInput("bend_angle", bend_angle);

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

  double rx = tf.transform.translation.x;
  double ry = tf.transform.translation.y;
  double ryaw = tf2::getYaw(tf.transform.rotation);

  double gx = goal.pose.position.x;
  double gy = goal.pose.position.y;

  // --- angle from robot heading to goal ---
  double to_goal = std::atan2(gy - ry, gx - rx);
  double rel = to_goal - ryaw;

  // normalise to [-pi, pi]
  while (rel >  M_PI) rel -= 2.0 * M_PI;
  while (rel < -M_PI) rel += 2.0 * M_PI;

  if (std::abs(rel) <= angle_thresh) {
    // Goal is in front — pass through unchanged
    setOutput("output_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

  // --- goal is behind: compute intermediate waypoint ---
  // Place it bend_dist ahead, offset bend_angle toward the goal side
  double offset = (rel > 0) ? bend_angle : -bend_angle;
  double heading = ryaw + offset;

  geometry_msgs::msg::PoseStamped bent;
  bent.header = goal.header;
  bent.header.stamp = node->get_clock()->now();
  bent.pose.position.x = rx + bend_dist * std::cos(heading);
  bent.pose.position.y = ry + bend_dist * std::sin(heading);
  bent.pose.position.z = 0.0;

  // Orient the waypoint toward the real goal
  double to_real = std::atan2(gy - bent.pose.position.y,
                              gx - bent.pose.position.x);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, to_real);
  bent.pose.orientation = tf2::toMsg(q);

  setOutput("output_goal", bent);

  RCLCPP_INFO(node->get_logger(),
    "GoalBender: goal behind (%.0f deg) -> intermediate (%.1f, %.1f)",
    std::abs(rel) * 180.0 / M_PI,
    bent.pose.position.x, bent.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace gradient_escape

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gradient_escape::GoalBender>("GoalBender");
}
