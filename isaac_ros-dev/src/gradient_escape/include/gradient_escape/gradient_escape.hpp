#ifndef GRADIENT_ESCAPE__GRADIENT_ESCAPE_HPP_
#define GRADIENT_ESCAPE__GRADIENT_ESCAPE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace gradient_escape
{

using DriveOnHeadingAction = nav2_msgs::action::DriveOnHeading;

/**
 * @brief NAV2 behavior that escapes high-cost costmap regions.
 *
 * When triggered, samples the local costmap in N directions around the
 * robot and drives toward the lowest cost.  Returns SUCCESS once the
 * robot's cell cost drops below a configurable threshold, or FAILED on
 * timeout.
 *
 * Reuses the DriveOnHeading action type so no custom msg package is
 * needed.  In the BT call it with server_name="gradient_escape".
 */
class GradientEscape : public nav2_behaviors::TimedBehavior<DriveOnHeadingAction>
{
public:
  GradientEscape();
  ~GradientEscape() override = default;

  Status onRun(const std::shared_ptr<const DriveOnHeadingAction::Goal> command) override;
  Status onCycleUpdate() override;
  void onConfigure() override;

protected:
  double escape_speed_;      // m/s  (default 0.1)
  double cost_threshold_;    // 0-254 (default 127)
  double sample_radius_;     // m    (default 0.15)
  int    num_samples_;       // directions to probe (default 16)
  double timeout_s_;         // seconds (default 15)

  rclcpp::Time start_time_;
  DriveOnHeadingAction::Feedback::SharedPtr feedback_;

  // Dedicated costmap subscriber for raw cell cost access
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
};

}  // namespace gradient_escape

#endif  // GRADIENT_ESCAPE__GRADIENT_ESCAPE_HPP_
