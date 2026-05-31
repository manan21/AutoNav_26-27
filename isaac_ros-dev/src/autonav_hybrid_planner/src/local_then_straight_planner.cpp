#include "autonav_hybrid_planner/local_then_straight_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

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
  start_search_radius_m_(1.00),
  start_blocked_cost_threshold_(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
  allow_unknown_start_(true),
  relax_blocked_start_(true),
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
    node, name_ + ".start_search_radius_m", rclcpp::ParameterValue(1.00));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_blocked_cost_threshold", rclcpp::ParameterValue(253));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown_start", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".relax_blocked_start", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".far_collision_check", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".near_planner_plugin", near_planner_plugin_);
  node->get_parameter(name_ + ".near_planner_name", near_planner_name_);
  node->get_parameter(name_ + ".local_horizon_m", local_horizon_m_);
  node->get_parameter(name_ + ".close_goal_distance_m", close_goal_distance_m_);
  node->get_parameter(name_ + ".far_path_spacing_m", far_path_spacing_m_);
  node->get_parameter(name_ + ".start_search_radius_m", start_search_radius_m_);
  int start_blocked_cost_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  node->get_parameter(name_ + ".start_blocked_cost_threshold", start_blocked_cost_threshold);
  node->get_parameter(name_ + ".allow_unknown_start", allow_unknown_start_);
  node->get_parameter(name_ + ".relax_blocked_start", relax_blocked_start_);
  node->get_parameter(name_ + ".far_collision_check", far_collision_check_);

  local_horizon_m_ = std::max(0.1, local_horizon_m_);
  close_goal_distance_m_ = std::max(local_horizon_m_, close_goal_distance_m_);
  far_path_spacing_m_ = std::max(0.05, far_path_spacing_m_);
  start_search_radius_m_ = std::max(0.0, start_search_radius_m_);
  start_blocked_cost_threshold_ = static_cast<unsigned char>(
    std::clamp(start_blocked_cost_threshold, 1, 255));
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
    "(local_horizon=%.2fm, close_goal=%.2fm, far_spacing=%.2fm, "
    "start_search_radius=%.2fm, start_blocked_threshold=%u, "
    "allow_unknown_start=%s, relax_blocked_start=%s, far_collision_check=%s)",
    name_.c_str(), near_planner_plugin_.c_str(), near_planner_full_name_.c_str(),
    local_horizon_m_, close_goal_distance_m_, far_path_spacing_m_,
    start_search_radius_m_, static_cast<unsigned int>(start_blocked_cost_threshold_),
    allow_unknown_start_ ? "true" : "false",
    relax_blocked_start_ ? "true" : "false",
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

  const geometry_msgs::msg::PoseStamped planning_start = choosePlanningStart(start);
  const double goal_distance = distanceBetween(planning_start, goal);
  if (goal_distance < 1e-3) {
    empty.poses.push_back(start);
    return empty;
  }

  if (goal_distance <= close_goal_distance_m_) {
    try {
      return prependActualStart(
        near_planner_->createPlan(planning_start, goal), start, planning_start);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(logger_, "Near-field planner failed for close goal: %s", ex.what());
      return empty;
    }
  }

  const auto handoff_goal = makeHandoffGoal(planning_start, goal, goal_distance);
  nav_msgs::msg::Path near_path;
  try {
    near_path = near_planner_->createPlan(planning_start, handoff_goal);
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

  return prependActualStart(appendStraightSegment(near_path, goal), start, planning_start);
}

bool LocalThenStraightPlanner::costIsBlocked(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !allow_unknown_start_;
  }
  return cost >= start_blocked_cost_threshold_;
}

bool LocalThenStraightPlanner::startCellIsBlocked(
  const geometry_msgs::msg::PoseStamped & start) const
{
  if (!costmap_) {
    return false;
  }
  return footprintIsBlocked(
    start.pose.position.x, start.pose.position.y, quaternionToYaw(start.pose.orientation));
}

double LocalThenStraightPlanner::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool LocalThenStraightPlanner::lineIsBlocked(
  double x0, double y0, double x1, double y1) const
{
  const double resolution = costmap_->getResolution();
  if (resolution <= 0.0) {
    return false;
  }
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / resolution)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double wx = x0 + (x1 - x0) * t;
    const double wy = y0 + (y1 - y0) * t;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      // Off the costmap: treat as unknown so an off-grid footprint edge does
      // not spuriously block the start (Smac handles unknown via allow_unknown).
      continue;
    }
    if (costIsBlocked(costmap_->getCost(mx, my))) {
      return true;
    }
  }
  return false;
}

bool LocalThenStraightPlanner::footprintIsBlocked(double wx, double wy, double yaw) const
{
  if (!costmap_) {
    return false;
  }

  // Use the same padded footprint Smac collision-checks against.
  const std::vector<geometry_msgs::msg::Point> footprint =
    costmap_ros_ ? costmap_ros_->getRobotFootprint() : std::vector<geometry_msgs::msg::Point>();

  if (footprint.size() < 3) {
    // Degenerate/circular footprint: fall back to a single-cell check.
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return false;
    }
    return costIsBlocked(costmap_->getCost(mx, my));
  }

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  auto to_world = [&](const geometry_msgs::msg::Point & p, double & ox, double & oy) {
    ox = wx + p.x * cos_yaw - p.y * sin_yaw;
    oy = wy + p.x * sin_yaw + p.y * cos_yaw;
  };

  // Check every edge of the oriented footprint polygon.
  for (size_t i = 0; i < footprint.size(); ++i) {
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    to_world(footprint[i], x0, y0);
    to_world(footprint[(i + 1) % footprint.size()], x1, y1);
    if (lineIsBlocked(x0, y0, x1, y1)) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::PoseStamped LocalThenStraightPlanner::choosePlanningStart(
  const geometry_msgs::msg::PoseStamped & start) const
{
  if (!relax_blocked_start_ || !costmap_ || start_search_radius_m_ <= 0.0 ||
    !startCellIsBlocked(start))
  {
    return start;
  }

  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    return start;
  }

  const double resolution = costmap_->getResolution();
  if (resolution <= 0.0) {
    return start;
  }
  const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap_->getSizeInCellsY());
  const int max_radius_cells = static_cast<int>(std::ceil(start_search_radius_m_ / resolution));
  const int sx = static_cast<int>(start_mx);
  const int sy = static_cast<int>(start_my);
  const double start_yaw = quaternionToYaw(start.pose.orientation);

  double best_dist_sq = std::numeric_limits<double>::infinity();
  int best_x = -1;
  int best_y = -1;
  for (int radius = 1; radius <= max_radius_cells; ++radius) {
    bool found_on_ring = false;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        const int mx = sx + dx;
        const int my = sy + dy;
        if (mx < 0 || my < 0 || mx >= size_x || my >= size_y) {
          continue;
        }
        // The candidate must be clear for the whole oriented footprint, not
        // just its center cell, otherwise Smac still rejects this start.
        double cell_wx = 0.0;
        double cell_wy = 0.0;
        costmap_->mapToWorld(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my), cell_wx, cell_wy);
        if (footprintIsBlocked(cell_wx, cell_wy, start_yaw)) {
          continue;
        }
        const double dist_sq = static_cast<double>(dx * dx + dy * dy);
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_x = mx;
          best_y = my;
          found_on_ring = true;
        }
      }
    }
    if (found_on_ring) {
      break;
    }
  }

  if (best_x < 0 || best_y < 0) {
    if (auto node = node_.lock()) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "Start pose is in cost >= %u and no free planning start was found within %.2f m",
        static_cast<unsigned int>(start_blocked_cost_threshold_), start_search_radius_m_);
    } else {
      RCLCPP_WARN(
        logger_,
        "Start pose is in cost >= %u and no free planning start was found within %.2f m",
        static_cast<unsigned int>(start_blocked_cost_threshold_), start_search_radius_m_);
    }
    return start;
  }

  geometry_msgs::msg::PoseStamped relaxed = start;
  costmap_->mapToWorld(
    static_cast<unsigned int>(best_x), static_cast<unsigned int>(best_y),
    relaxed.pose.position.x, relaxed.pose.position.y);
  if (auto node = node_.lock()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "Start pose is in cost >= %u; planning from nearest clear cell %.2f m away",
      static_cast<unsigned int>(start_blocked_cost_threshold_),
      std::sqrt(best_dist_sq) * resolution);
  } else {
    RCLCPP_WARN(
      logger_,
      "Start pose is in cost >= %u; planning from nearest clear cell %.2f m away",
      static_cast<unsigned int>(start_blocked_cost_threshold_),
      std::sqrt(best_dist_sq) * resolution);
  }
  return relaxed;
}

nav_msgs::msg::Path LocalThenStraightPlanner::prependActualStart(
  nav_msgs::msg::Path path,
  const geometry_msgs::msg::PoseStamped & actual_start,
  const geometry_msgs::msg::PoseStamped & planning_start) const
{
  if (path.poses.empty() || distanceBetween(actual_start, planning_start) < 1e-3) {
    return path;
  }
  auto start_pose = actual_start;
  start_pose.header.frame_id = path.header.frame_id.empty()
    ? actual_start.header.frame_id : path.header.frame_id;
  start_pose.header.stamp = path.header.stamp;
  path.poses.insert(path.poses.begin(), start_pose);
  return path;
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
