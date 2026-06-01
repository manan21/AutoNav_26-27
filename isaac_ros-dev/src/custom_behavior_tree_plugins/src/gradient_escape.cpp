#include "gradient_escape/gradient_escape.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gradient_escape
{

GradientEscape::GradientEscape()
: TimedBehavior<DriveOnHeadingAction>(),
  escape_speed_(0.1),
  max_search_radius_m_(2.0),
  timeout_s_(15.0),
  plan_topic_("/plan")
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
    node, "gradient_escape.max_search_radius_m", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.timeout", rclcpp::ParameterValue(15.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "gradient_escape.plan_topic", rclcpp::ParameterValue(std::string("/plan")));

  node->get_parameter("gradient_escape.escape_speed", escape_speed_);
  node->get_parameter("gradient_escape.max_search_radius_m", max_search_radius_m_);
  node->get_parameter("gradient_escape.timeout", timeout_s_);
  node->get_parameter("gradient_escape.plan_topic", plan_topic_);

  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node, "local_costmap/costmap_raw");

  plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    plan_topic_, rclcpp::QoS(1).reliable(),
    [this](nav_msgs::msg::Path::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      latest_plan_ = msg;
    });

  feedback_ = std::make_shared<DriveOnHeadingAction::Feedback>();
}

Status GradientEscape::onRun(
  const std::shared_ptr<const DriveOnHeadingAction::Goal> /*command*/)
{
  start_time_ = clock_->now();
  RCLCPP_INFO(
    logger_,
    "GradientEscape: starting (speed=%.2f m/s, search_radius=%.2f m, timeout=%.1f s)",
    escape_speed_, max_search_radius_m_, timeout_s_);
  return Status::SUCCEEDED;  // proceed into onCycleUpdate loop
}

Status GradientEscape::onCycleUpdate()
{
  const double elapsed = (clock_->now() - start_time_).seconds();
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(logger_, "GradientEscape: timed out after %.1f s", timeout_s_);
    stopRobot();
    return Status::FAILED;
  }

  // ---- Primary exit: /plan is back live ----
  // A non-empty path published after this behavior started means the
  // planner can route again. This only fires if a concurrent re-planning
  // mechanism is in place; otherwise we fall through to the cost check.
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (latest_plan_ && !latest_plan_->poses.empty()) {
      const rclcpp::Time stamp(latest_plan_->header.stamp, clock_->get_clock_type());
      if (stamp.nanoseconds() > 0 && stamp > start_time_) {
        RCLCPP_INFO(
          logger_,
          "GradientEscape: /plan back live (%zu poses) after %.1f s, exiting",
          latest_plan_->poses.size(), elapsed);
        stopRobot();
        return Status::SUCCEEDED;
      }
    }
  }

  // ---- Robot pose in the costmap frame ----
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "GradientEscape: cannot get robot pose");
    stopRobot();
    return Status::FAILED;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "GradientEscape: costmap not available yet (%s)", e.what());
    return Status::RUNNING;
  }

  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;

  unsigned int rmx = 0;
  unsigned int rmy = 0;
  if (!costmap->worldToMap(rx, ry, rmx, rmy)) {
    RCLCPP_WARN(logger_, "GradientEscape: robot outside local costmap");
    stopRobot();
    return Status::FAILED;
  }

  // ---- Fallback exit: robot cell cost is below the planner's blocking
  // threshold. NavfnPlanner blocks at cost >= INSCRIBED_INFLATED_OBSTACLE
  // (253), so once the robot's cell is below that the planner can seed
  // a path on the next BT cycle. This is the realistic exit for the
  // current BT structure where the planner only re-fires after this
  // behavior returns SUCCESS. ----
  const unsigned char robot_cost = costmap->getCost(rmx, rmy);
  if (robot_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_INFO(
      logger_,
      "GradientEscape: robot cell cost %u below planner threshold (253) after %.1f s, exiting",
      static_cast<unsigned int>(robot_cost), elapsed);
    stopRobot();
    return Status::SUCCEEDED;
  }

  // ---- Search for the nearest lethal cell within max_search_radius_m_ ----
  const double resolution = costmap->getResolution();
  const int search_cells = static_cast<int>(
    std::ceil(max_search_radius_m_ / resolution));
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  const int min_mx = std::max(0, static_cast<int>(rmx) - search_cells);
  const int max_mx = std::min(size_x - 1, static_cast<int>(rmx) + search_cells);
  const int min_my = std::max(0, static_cast<int>(rmy) - search_cells);
  const int max_my = std::min(size_y - 1, static_cast<int>(rmy) + search_cells);

  int best_dx = 0;
  int best_dy = 0;
  int best_d2 = std::numeric_limits<int>::max();
  bool found = false;

  for (int my = min_my; my <= max_my; ++my) {
    const int dy = my - static_cast<int>(rmy);
    const int dy2 = dy * dy;
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      const unsigned char c = costmap->getCost(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
      if (c < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      const int dx = mx - static_cast<int>(rmx);
      const int d2 = dx * dx + dy2;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_dx = dx;
        best_dy = dy;
        found = true;
      }
    }
  }

  if (!found) {
    // Nothing lethal in range yet robot is in cost >= 253. This usually
    // means the body is sitting inside an inflated region with the actual
    // lethal source just outside the bounded search. Widening the search
    // is cheap; alternatively the recovery falls through to ClearCostmap
    // on the next BT cycle.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "GradientEscape: no lethal cell within %.1f m, holding position",
      max_search_radius_m_);
    stopRobot();
    return Status::RUNNING;
  }

  // best_dx, best_dy point FROM the robot TO the nearest lethal cell
  // (cell deltas). Robot escapes by moving in the opposite direction,
  // so the escape angle is atan2(-dy, -dx). If multiple lethal cells
  // are equally close, the search picks one arbitrarily — the only
  // pinch case where that matters (robot between two equidistant
  // obstacles) is rare given IGVC's 5 ft minimum obstacle spacing.
  const double escape_angle = std::atan2(
    static_cast<double>(-best_dy), static_cast<double>(-best_dx));

  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  double rel = escape_angle - robot_yaw;
  while (rel > M_PI) {rel -= 2.0 * M_PI;}
  while (rel < -M_PI) {rel += 2.0 * M_PI;}

  auto cmd = std::make_unique<geometry_msgs::msg::Twist>();

  if (std::abs(rel) <= M_PI_2) {
    // Escape direction is in the front half-plane: drive forward and
    // steer toward it. Proportional gain 1.5 with a 1.0 rad/s clamp
    // gives smooth alignment without jerk.
    cmd->linear.x = escape_speed_;
    cmd->angular.z = std::clamp(rel * 1.5, -1.0, 1.0);
  } else {
    // Escape direction is in the rear half-plane: reverse straight along
    // the body axis. For a diff-drive sitting next to an obstacle, this
    // is safer than rotating in place (which sweeps the body sideways
    // through the obstacle). Steering compensates for the alignment
    // between body-rear and the escape direction.
    cmd->linear.x = -escape_speed_;
    double rel_rear = rel - std::copysign(M_PI, rel);  // angle between
                                                       // body-rear and
                                                       // escape direction
    cmd->angular.z = std::clamp(rel_rear * 1.5, -1.0, 1.0);
  }

  vel_pub_->publish(std::move(cmd));

  feedback_->distance_traveled = static_cast<float>(elapsed);
  action_server_->publish_feedback(feedback_);

  return Status::RUNNING;
}

}  // namespace gradient_escape

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_escape::GradientEscape, nav2_core::Behavior)
