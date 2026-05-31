#include "autonav_hybrid_planner/local_then_straight_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace autonav_hybrid_planner
{

namespace
{

double distanceBetween(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = b.pose.position.x - a.pose.position.x;
  const double dy = b.pose.position.y - a.pose.position.y;
  return std::hypot(dx, dy);
}

}  // namespace

LocalThenStraightPlanner::LocalThenStraightPlanner()
: near_planner_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  costmap_(nullptr),
  logger_(rclcpp::get_logger("LocalThenStraightPlanner")),
  local_horizon_m_(2.75),
  close_goal_distance_m_(3.0),
  far_path_spacing_m_(0.50),
  far_collision_check_(false),
  configured_(false)
{
}

void LocalThenStraightPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  global_frame_ = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : std::string("map");

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LocalThenStraightPlanner: parent lifecycle node expired");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".near_planner_plugin",
    rclcpp::ParameterValue("nav2_smac_planner/SmacPlannerLattice"));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".near_planner_name",
    rclcpp::ParameterValue("near_lattice"));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".local_horizon_m", rclcpp::ParameterValue(2.75));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".close_goal_distance_m", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".far_path_spacing_m", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".far_collision_check", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".near_planner_plugin", near_planner_plugin_);
  node->get_parameter(name_ + ".near_planner_name", near_planner_name_);
  node->get_parameter(name_ + ".local_horizon_m", local_horizon_m_);
  node->get_parameter(name_ + ".close_goal_distance_m", close_goal_distance_m_);
  node->get_parameter(name_ + ".far_path_spacing_m", far_path_spacing_m_);
  node->get_parameter(name_ + ".far_collision_check", far_collision_check_);

  local_horizon_m_ = std::max(0.1, local_horizon_m_);
  close_goal_distance_m_ = std::max(local_horizon_m_, close_goal_distance_m_);
  far_path_spacing_m_ = std::max(0.05, far_path_spacing_m_);
  near_planner_full_name_ = name_ + "." + near_planner_name_;

  try {
    near_planner_ = near_planner_loader_.createSharedInstance(near_planner_plugin_);
    near_planner_->configure(parent, near_planner_full_name_, tf_, costmap_ros_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create near-field planner '%s': %s",
      near_planner_plugin_.c_str(), ex.what());
    throw;
  }

  configured_ = true;
  RCLCPP_INFO(
    logger_,
    "Configured %s with near planner %s at namespace %s "
    "(local_horizon=%.2fm, close_goal=%.2fm, far_spacing=%.2fm, far_collision_check=%s)",
    name_.c_str(), near_planner_plugin_.c_str(), near_planner_full_name_.c_str(),
    local_horizon_m_, close_goal_distance_m_, far_path_spacing_m_,
    far_collision_check_ ? "true" : "false");
}

void LocalThenStraightPlanner::cleanup()
{
  if (near_planner_) {
    near_planner_->cleanup();
    near_planner_.reset();
  }
  configured_ = false;
}

void LocalThenStraightPlanner::activate()
{
  if (near_planner_) {
    near_planner_->activate();
  }
}

void LocalThenStraightPlanner::deactivate()
{
  if (near_planner_) {
    near_planner_->deactivate();
  }
}

nav_msgs::msg::Path LocalThenStraightPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = start.header.frame_id.empty() ? global_frame_ : start.header.frame_id;
  if (auto node = node_.lock()) {
    empty.header.stamp = node->now();
  } else {
    empty.header.stamp = start.header.stamp;
  }

  if (!configured_ || !near_planner_) {
    RCLCPP_ERROR(logger_, "LocalThenStraightPlanner called before configure()");
    return empty;
  }

  if (start.header.frame_id != goal.header.frame_id) {
    RCLCPP_ERROR(
      logger_,
      "Start frame '%s' does not match goal frame '%s'",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return empty;
  }

  const double goal_distance = distanceBetween(start, goal);
  if (goal_distance < 1e-3) {
    empty.poses.push_back(start);
    return empty;
  }

  if (goal_distance <= close_goal_distance_m_) {
    try {
      return near_planner_->createPlan(start, goal);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(logger_, "Near-field planner failed for close goal: %s", ex.what());
      return empty;
    }
  }

  const auto handoff_goal = makeHandoffGoal(start, goal, goal_distance);
  nav_msgs::msg::Path near_path;
  try {
    near_path = near_planner_->createPlan(start, handoff_goal);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger_, "Near-field planner failed for far goal: %s", ex.what());
    return empty;
  }

  if (near_path.poses.empty()) {
    RCLCPP_WARN(logger_, "Near-field planner returned an empty path");
    return empty;
  }

  if (near_path.header.frame_id.empty()) {
    near_path.header.frame_id = empty.header.frame_id;
  }
  if (near_path.header.stamp.sec == 0 && near_path.header.stamp.nanosec == 0) {
    near_path.header.stamp = empty.header.stamp;
  }

  if (far_collision_check_ && !straightSegmentIsAllowed(near_path.poses.back(), goal)) {
    RCLCPP_WARN(
      logger_,
      "Far straight segment rejected by centerline collision check");
    return empty;
  }

  return appendStraightSegment(near_path, goal);
}

geometry_msgs::msg::PoseStamped LocalThenStraightPlanner::makeHandoffGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double distance) const
{
  geometry_msgs::msg::PoseStamped handoff = goal;
  const double ratio = local_horizon_m_ / distance;
  handoff.pose.position.x =
    start.pose.position.x + (goal.pose.position.x - start.pose.position.x) * ratio;
  handoff.pose.position.y =
    start.pose.position.y + (goal.pose.position.y - start.pose.position.y) * ratio;
  handoff.pose.position.z =
    start.pose.position.z + (goal.pose.position.z - start.pose.position.z) * ratio;
  handoff.pose.orientation = yawToQuaternion(yawBetween(start, goal));
  return handoff;
}

nav_msgs::msg::Path LocalThenStraightPlanner::appendStraightSegment(
  nav_msgs::msg::Path near_path,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (near_path.poses.empty()) {
    return near_path;
  }

  const auto from = near_path.poses.back();
  const double straight_distance = distanceBetween(from, goal);
  if (straight_distance < 1e-3) {
    return near_path;
  }

  const int steps = std::max(
    1, static_cast<int>(std::ceil(straight_distance / far_path_spacing_m_)));
  const auto orientation = yawToQuaternion(yawBetween(from, goal));

  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::msg::PoseStamped pose = goal;
    pose.header.frame_id = near_path.header.frame_id;
    pose.header.stamp = near_path.header.stamp;
    pose.pose.position.x =
      from.pose.position.x + (goal.pose.position.x - from.pose.position.x) * t;
    pose.pose.position.y =
      from.pose.position.y + (goal.pose.position.y - from.pose.position.y) * t;
    pose.pose.position.z =
      from.pose.position.z + (goal.pose.position.z - from.pose.position.z) * t;
    pose.pose.orientation = orientation;
    near_path.poses.push_back(pose);
  }

  return near_path;
}

bool LocalThenStraightPlanner::straightSegmentIsAllowed(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (!costmap_) {
    return false;
  }

  const double straight_distance = distanceBetween(from, goal);
  const int steps = std::max(
    1, static_cast<int>(std::ceil(straight_distance / far_path_spacing_m_)));

  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double wx = from.pose.position.x + (goal.pose.position.x - from.pose.position.x) * t;
    const double wy = from.pose.position.y + (goal.pose.position.y - from.pose.position.y) * t;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      continue;
    }
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return false;
    }
  }

  return true;
}

double LocalThenStraightPlanner::yawBetween(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to)
{
  return std::atan2(
    to.pose.position.y - from.pose.position.y,
    to.pose.position.x - from.pose.position.x);
}

geometry_msgs::msg::Quaternion LocalThenStraightPlanner::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace autonav_hybrid_planner

PLUGINLIB_EXPORT_CLASS(
  autonav_hybrid_planner::LocalThenStraightPlanner,
  nav2_core::GlobalPlanner)
