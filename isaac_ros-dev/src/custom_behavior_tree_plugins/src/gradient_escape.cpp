#include "gradient_escape/gradient_escape.hpp"

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace gradient_escape
{

GradientEscape::GradientEscape()
: TimedBehavior<DriveOnHeadingAction>(),
  escape_speed_(0.1),
  cost_threshold_(127),
  sample_radius_(0.15),
  num_samples_(16),
  timeout_s_(15.0)
{
}

void GradientEscape::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GradientEscape: failed to lock node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.escape_speed", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.cost_threshold", rclcpp::ParameterValue(127.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.sample_radius", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.num_samples", rclcpp::ParameterValue(16));
  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.timeout", rclcpp::ParameterValue(15.0));

  node->get_parameter("gradient_escape.escape_speed", escape_speed_);
  node->get_parameter("gradient_escape.cost_threshold", cost_threshold_);
  node->get_parameter("gradient_escape.sample_radius", sample_radius_);
  node->get_parameter("gradient_escape.num_samples", num_samples_);
  node->get_parameter("gradient_escape.timeout", timeout_s_);

  // Subscribe to the local costmap for raw cell costs
  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node, "local_costmap/costmap_raw");

  feedback_ = std::make_shared<DriveOnHeadingAction::Feedback>();
}

Status GradientEscape::onRun(
  const std::shared_ptr<const DriveOnHeadingAction::Goal> /*command*/)
{
  start_time_ = clock_->now();
  RCLCPP_INFO(logger_, "GradientEscape: starting (threshold=%d, speed=%.2f, timeout=%.1fs)",
    static_cast<int>(cost_threshold_), escape_speed_, timeout_s_);
  return Status::SUCCEEDED;  // proceed to onCycleUpdate loop
}

Status GradientEscape::onCycleUpdate()
{
  // ---- timeout check ----
  double elapsed = (clock_->now() - start_time_).seconds();
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(logger_, "GradientEscape: timed out after %.1fs", timeout_s_);
    stopRobot();
    return Status::FAILED;
  }

  // ---- get robot pose in the costmap frame (odom) ----
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
        pose, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "GradientEscape: cannot get robot pose");
    stopRobot();
    return Status::FAILED;
  }

  double rx = pose.pose.position.x;
  double ry = pose.pose.position.y;

  // ---- read costmap ----
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "GradientEscape: costmap not available yet (%s)", e.what());
    return Status::RUNNING;  // keep waiting
  }

  unsigned int mx, my;
  if (!costmap->worldToMap(rx, ry, mx, my)) {
    RCLCPP_WARN(logger_, "GradientEscape: robot outside costmap");
    stopRobot();
    return Status::FAILED;
  }

  unsigned char robot_cost = costmap->getCost(mx, my);

  // ---- success check ----
  if (robot_cost < static_cast<unsigned char>(cost_threshold_)) {
    RCLCPP_INFO(logger_, "GradientEscape: escaped (cost %d < %d) in %.1fs",
      static_cast<int>(robot_cost), static_cast<int>(cost_threshold_), elapsed);
    stopRobot();
    return Status::SUCCEEDED;
  }

  // ---- sample gradient ----
  double best_angle = 0.0;
  unsigned char best_cost = 255;
  bool found = false;

  for (int i = 0; i < num_samples_; ++i) {
    double angle = 2.0 * M_PI * i / num_samples_;
    double sx = rx + sample_radius_ * std::cos(angle);
    double sy = ry + sample_radius_ * std::sin(angle);

    unsigned int smx, smy;
    if (!costmap->worldToMap(sx, sy, smx, smy)) {
      continue;
    }

    unsigned char c = costmap->getCost(smx, smy);
    if (c < best_cost) {
      best_cost = c;
      best_angle = angle;
      found = true;
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "GradientEscape: no viable direction");
    stopRobot();
    return Status::FAILED;
  }

  // ---- command velocity toward lowest cost ----
  double robot_yaw = tf2::getYaw(pose.pose.orientation);
  double rel = best_angle - robot_yaw;

  // normalise to [-pi, pi]
  while (rel >  M_PI) rel -= 2.0 * M_PI;
  while (rel < -M_PI) rel += 2.0 * M_PI;

  auto cmd = std::make_unique<geometry_msgs::msg::Twist>();

  if (std::abs(rel) < M_PI / 2.0) {
    // Escape direction is roughly ahead — drive with proportional steering
    cmd->linear.x = escape_speed_;
    cmd->angular.z = std::clamp(rel * 1.5, -1.0, 1.0);
  } else {
    // Escape direction is behind — rotate in place first
    cmd->linear.x = 0.0;
    cmd->angular.z = std::copysign(0.5, rel);
  }

  vel_pub_->publish(std::move(cmd));

  feedback_->distance_traveled = static_cast<float>(elapsed);
  action_server_->publish_feedback(feedback_);

  return Status::RUNNING;
}

}  // namespace gradient_escape

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_escape::GradientEscape, nav2_core::Behavior)
