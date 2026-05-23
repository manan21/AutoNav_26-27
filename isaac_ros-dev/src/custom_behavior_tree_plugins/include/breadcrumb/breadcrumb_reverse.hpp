#ifndef BREADCRUMB__BREADCRUMB_REVERSE_HPP_
#define BREADCRUMB__BREADCRUMB_REVERSE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace breadcrumb
{

using DriveOnHeadingAction = nav2_msgs::action::DriveOnHeading;
using nav2_behaviors::Status;

// nav2 behavior plugin: reverses the robot toward the most recent
// breadcrumb on /breadcrumb_tail. Reuses the DriveOnHeading action
// type (the BT calls us via <DriveOnHeading server_name="breadcrumb_reverse"/>)
// and ignores the goal contents — the breadcrumb buffer is the source
// of truth for the target.
//
// Safety: queries /global_costmap/costmap_raw at the target breadcrumb
// pose before committing. Global costmap retains rear obstacles via
// local_mirror_layer (see nav2_paramsv2.yaml), so the robot can check
// what it lidar-saw on the way forward.
//
// Returns:
//   SUCCEEDED — robot reached the breadcrumb within arrival_tolerance_m
//   FAILED    — buffer empty, target in lethal global cost, pose lookup
//               failed, or timeout exceeded
class BreadcrumbReverse : public nav2_behaviors::TimedBehavior<DriveOnHeadingAction>
{
public:
  BreadcrumbReverse();
  ~BreadcrumbReverse() override = default;

  Status onRun(const std::shared_ptr<const DriveOnHeadingAction::Goal> command) override;
  Status onCycleUpdate() override;
  void onConfigure() override;

protected:
  double reverse_speed_;
  double max_angular_speed_;
  double arrival_tolerance_m_;
  double lethal_cost_threshold_;
  double timeout_s_;
  std::string breadcrumb_topic_;
  std::string global_costmap_topic_;
  // Sim-port params:
  // `max_crumbs_per_session`: cap on breadcrumbs consumed across
  //   back-to-back invocations. Once hit, the next invocation returns
  //   FAILED — the BT's ReactiveFallback then drops into Wait and the
  //   next RateController tick fires GoalBender (away-from-costmap).
  // `session_reset_s`: idle gap before the session counter resets.
  // `bonus_crumb_after_forward`: when true, after a successful pop the
  //   plugin sniffs /plan; if the path's first useful segment is now
  //   `forward_threshold_rad` clear of the body's +x, the plugin pops
  //   ONE MORE crumb (the bonus margin) before returning SUCCEEDED.
  // `forward_threshold_rad`: tighter than the BT's `angle_threshold`
  //   so we only chain the bonus crumb when the plan is solidly
  //   forward (default 60° = π/3).
  int max_crumbs_per_session_;
  double session_reset_s_;
  bool bonus_crumb_after_forward_;
  double forward_threshold_rad_;
  double min_lookahead_m_;
  std::string plan_topic_;

  rclcpp::Time start_time_;
  DriveOnHeadingAction::Feedback::SharedPtr feedback_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr breadcrumb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> global_costmap_sub_;

  std::mutex tail_mutex_;
  nav_msgs::msg::Path latest_tail_;
  std::mutex plan_mutex_;
  nav_msgs::msg::Path latest_plan_;

  // Captured at onRun time so the same target is pursued across cycles
  // (a fresh tail msg arriving mid-action shouldn't redirect us).
  geometry_msgs::msg::PoseStamped target_;
  bool have_target_{false};

  // Bonus-crumb state inside a single onRun:
  //   first arrival w/ forward plan → set pending; capture next crumb;
  //   second arrival w/ pending → SUCCEEDED.
  bool bonus_pending_{false};

  // Session state across consecutive onRun invocations (the BT's
  // ReactiveFallback loops us). Reset when more than session_reset_s
  // has passed without an invocation.
  int crumbs_consumed_session_{0};
  rclcpp::Time last_invocation_end_;
};

}  // namespace breadcrumb

#endif  // BREADCRUMB__BREADCRUMB_REVERSE_HPP_
