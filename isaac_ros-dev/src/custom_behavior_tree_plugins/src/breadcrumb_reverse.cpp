#include "breadcrumb/breadcrumb_reverse.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "breadcrumb/forward_blocked_check.hpp"

namespace breadcrumb
{

BreadcrumbReverse::BreadcrumbReverse()
: nav2_behaviors::TimedBehavior<DriveOnHeadingAction>(),
  reverse_speed_(0.10),
  max_angular_speed_(0.50),
  arrival_tolerance_m_(0.05),
  lethal_cost_threshold_(253.0),
  timeout_s_(5.0),
  breadcrumb_topic_("/breadcrumb_tail"),
  global_costmap_topic_("global_costmap/costmap_raw"),
  max_crumbs_per_session_(15),
  session_reset_s_(2.0),
  bonus_crumb_after_forward_(true),
  forward_threshold_rad_(M_PI / 3.0),   // 60°
  min_lookahead_m_(0.6),
  plan_topic_("/plan")
{
}

void BreadcrumbReverse::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("BreadcrumbReverse: failed to lock node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.reverse_speed", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.max_angular_speed", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.arrival_tolerance_m", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.lethal_cost_threshold", rclcpp::ParameterValue(253.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.timeout", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.breadcrumb_topic",
    rclcpp::ParameterValue(std::string("/breadcrumb_tail")));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.global_costmap_topic",
    rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
  // Sim-port params.
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.max_crumbs_per_session", rclcpp::ParameterValue(15));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.session_reset_s", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.bonus_crumb_after_forward", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.forward_threshold_deg", rclcpp::ParameterValue(60.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.min_lookahead_m", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, "breadcrumb_reverse.plan_topic",
    rclcpp::ParameterValue(std::string("/plan")));

  node->get_parameter("breadcrumb_reverse.reverse_speed", reverse_speed_);
  node->get_parameter("breadcrumb_reverse.max_angular_speed", max_angular_speed_);
  node->get_parameter("breadcrumb_reverse.arrival_tolerance_m", arrival_tolerance_m_);
  node->get_parameter("breadcrumb_reverse.lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter("breadcrumb_reverse.timeout", timeout_s_);
  node->get_parameter("breadcrumb_reverse.breadcrumb_topic", breadcrumb_topic_);
  node->get_parameter("breadcrumb_reverse.global_costmap_topic", global_costmap_topic_);
  node->get_parameter("breadcrumb_reverse.max_crumbs_per_session", max_crumbs_per_session_);
  node->get_parameter("breadcrumb_reverse.session_reset_s", session_reset_s_);
  node->get_parameter("breadcrumb_reverse.bonus_crumb_after_forward",
                      bonus_crumb_after_forward_);
  double forward_threshold_deg = 60.0;
  node->get_parameter("breadcrumb_reverse.forward_threshold_deg",
                      forward_threshold_deg);
  forward_threshold_rad_ = forward_threshold_deg * M_PI / 180.0;
  node->get_parameter("breadcrumb_reverse.min_lookahead_m", min_lookahead_m_);
  node->get_parameter("breadcrumb_reverse.plan_topic", plan_topic_);

  breadcrumb_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    breadcrumb_topic_,
    rclcpp::QoS(1).transient_local().reliable(),
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(tail_mutex_);
      latest_tail_ = *msg;
    });

  // Subscribe to whichever plan topic the planner_server is publishing
  // on. Used by the bonus-crumb logic to peek at the current plan after
  // each successful pop — if the first useful segment is solidly
  // forward we chain ONE MORE crumb (margin) before returning SUCCEEDED.
  plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    plan_topic_,
    rclcpp::QoS(1).best_effort(),
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(plan_mutex_);
      latest_plan_ = *msg;
    });

  global_costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node, global_costmap_topic_);

  feedback_ = std::make_shared<DriveOnHeadingAction::Feedback>();
  last_invocation_end_ = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
}

Status BreadcrumbReverse::onRun(
  const std::shared_ptr<const DriveOnHeadingAction::Goal> /*command*/)
{
  start_time_ = clock_->now();
  have_target_ = false;
  bonus_pending_ = false;

  // Session counter — if we've been idle for >session_reset_s, treat
  // this as a NEW session and reset the count. Otherwise the count
  // continues across the BT's ReactiveFallback loop calls.
  const double idle = (start_time_ - last_invocation_end_).seconds();
  if (idle > session_reset_s_) {
    crumbs_consumed_session_ = 0;
  }
  if (crumbs_consumed_session_ >= max_crumbs_per_session_) {
    RCLCPP_INFO(logger_,
      "BreadcrumbReverse: session cap reached (%d crumbs); returning FAILED "
      "so GoalBender can take over",
      max_crumbs_per_session_);
    crumbs_consumed_session_ = 0;        // arm fresh count for next time
    return Status::FAILED;
  }

  // Capture the breadcrumb to chase. Use the most recent (back of the
  // deque published as poses.back()). Snapshot now so the buffer
  // popping ahead of us doesn't redirect mid-action.
  {
    std::lock_guard<std::mutex> lk(tail_mutex_);
    if (latest_tail_.poses.empty()) {
      RCLCPP_INFO(logger_, "BreadcrumbReverse: buffer empty, nothing to reverse to");
      return Status::FAILED;
    }
    target_ = latest_tail_.poses.back();
    have_target_ = true;
  }

  // Sanity-check the captured target against the global costmap.
  // local_mirror_layer in the global costmap accumulates rear lidar
  // marks from when the robot drove forward past them, so a lethal
  // hit here means an obstacle is sitting where we'd reverse to.
  try {
    auto global_cm = global_costmap_sub_->getCostmap();
    unsigned int mx, my;
    if (global_cm->worldToMap(
          target_.pose.position.x, target_.pose.position.y, mx, my))
    {
      const unsigned char c = global_cm->getCost(mx, my);
      if (static_cast<double>(c) >= lethal_cost_threshold_) {
        RCLCPP_WARN(logger_,
          "BreadcrumbReverse: target (%.2f, %.2f) has lethal global cost %d, aborting",
          target_.pose.position.x, target_.pose.position.y, static_cast<int>(c));
        return Status::FAILED;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_,
      "BreadcrumbReverse: global costmap unavailable for safety check (%s); proceeding",
      e.what());
  }

  RCLCPP_INFO(logger_,
    "BreadcrumbReverse: reversing to breadcrumb (%.2f, %.2f) in frame '%s'",
    target_.pose.position.x, target_.pose.position.y,
    target_.header.frame_id.c_str());

  return Status::SUCCEEDED;  // proceed to onCycleUpdate loop
}

Status BreadcrumbReverse::onCycleUpdate()
{
  const double elapsed = (clock_->now() - start_time_).seconds();
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(logger_, "BreadcrumbReverse: timed out after %.1fs", timeout_s_);
    stopRobot();
    return Status::FAILED;
  }

  if (!have_target_) {
    stopRobot();
    return Status::FAILED;
  }

  // Robot pose in the same frame as the breadcrumb (odom by buffer
  // construction). global_frame_ on the behavior_server is "odom" — see
  // the active Nav2 params — so getCurrentPose returns the matching
  // frame without extra TF math.
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
        pose, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "BreadcrumbReverse: cannot get robot pose");
    stopRobot();
    return Status::FAILED;
  }

  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;
  const double ryaw = tf2::getYaw(pose.pose.orientation);

  const double tx = target_.pose.position.x;
  const double ty = target_.pose.position.y;

  const double dx = tx - rx;
  const double dy = ty - ry;
  const double dist = std::hypot(dx, dy);

  if (dist <= arrival_tolerance_m_) {
    ++crumbs_consumed_session_;
    last_invocation_end_ = clock_->now();
    RCLCPP_INFO(logger_,
      "BreadcrumbReverse: arrived at (%.2f, %.2f) in %.1fs "
      "[session_count=%d/%d, bonus_pending=%d]",
      tx, ty, elapsed,
      crumbs_consumed_session_, max_crumbs_per_session_,
      static_cast<int>(bonus_pending_));

    // Sim-port bonus-crumb logic: if the freshest /plan is now solidly
    // forward (carrot bearing < forward_threshold_rad), the BT's outer
    // loop will exit BREADCRUMB. To give the controller MARGIN from the
    // costmap halo, chain ONE more reverse to the next crumb before
    // declaring done. The second arrival (bonus_pending_ already set)
    // is the actual exit point.
    if (bonus_crumb_after_forward_ && !bonus_pending_) {
      nav_msgs::msg::Path plan_snapshot;
      {
        std::lock_guard<std::mutex> lk(plan_mutex_);
        plan_snapshot = latest_plan_;
      }
      const double err = computeFirstSegBearingErrSigned(
        rx, ry, ryaw, plan_snapshot, min_lookahead_m_);
      const bool solidly_forward =
        std::isfinite(err) && std::abs(err) < forward_threshold_rad_;
      if (solidly_forward) {
        // Try to grab the next-most-recent crumb (now poses.back() after
        // the buffer popped). If no next crumb exists, just exit clean.
        std::lock_guard<std::mutex> lk(tail_mutex_);
        if (!latest_tail_.poses.empty()) {
          target_ = latest_tail_.poses.back();
          have_target_ = true;
          bonus_pending_ = true;
          // Reset action timer so the bonus crumb gets a fresh allowance.
          start_time_ = clock_->now();
          RCLCPP_INFO(logger_,
            "BreadcrumbReverse: plan is solidly forward (err=%.2frad); "
            "chaining bonus crumb at (%.2f, %.2f)",
            err, target_.pose.position.x, target_.pose.position.y);
          return Status::RUNNING;
        }
      }
    }

    stopRobot();
    return Status::SUCCEEDED;
  }

  // Heading from robot to target in odom; convert to body-frame relative
  // angle. For a reverse maneuver we want the *back* of the robot to
  // point at the target, i.e. relative body angle ≈ ±π. The angular
  // command rotates that error toward π so the rear tracks the target
  // as we drive backward.
  const double to_target_world = std::atan2(dy, dx);
  const double rel = wrap_pi(to_target_world - ryaw);

  // err = how far rel is from π (i.e. how off the rear-facing alignment
  // is). Sign of err picks rotation direction.
  double err = wrap_pi(M_PI - rel);
  // Smaller err → less rotation. Drive backward at full speed once
  // mostly aligned; back off speed when err is large so we don't carve
  // a wide arc.
  const double align_factor = std::clamp(
    1.0 - std::abs(err) / (M_PI / 2.0), 0.0, 1.0);

  auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
  cmd->linear.x = -reverse_speed_ * align_factor;
  cmd->angular.z = std::clamp(-err * 1.5, -max_angular_speed_, max_angular_speed_);

  vel_pub_->publish(std::move(cmd));

  feedback_->distance_traveled = static_cast<float>(dist);
  action_server_->publish_feedback(feedback_);

  return Status::RUNNING;
}

}  // namespace breadcrumb

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(breadcrumb::BreadcrumbReverse, nav2_core::Behavior)
