#ifndef GRADIENT_ESCAPE__GRADIENT_ESCAPE_HPP_
#define GRADIENT_ESCAPE__GRADIENT_ESCAPE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace gradient_escape
{

using DriveOnHeadingAction = nav2_msgs::action::DriveOnHeading;
using nav2_behaviors::Status;

/**
 * @brief Emergency-only recovery: drive the robot away from the nearest
 * lethal costmap cell until the planner can produce a path again.
 *
 * This is not a true gradient descent. It scans the local costmap for the
 * nearest cell at cost >= LETHAL_OBSTACLE, computes the radial vector from
 * that cell to the robot center, and commands velocity along that vector
 * (forward if it points ahead, reverse if behind). Two exit conditions:
 *
 *  - Primary: a /plan message arrives with a stamp later than the moment
 *    this behavior started AND non-empty poses ("path is back live").
 *    This only fires if something is actively re-firing the planner during
 *    recovery (e.g. a Parallel BT node or a side-channel re-planner).
 *  - Fallback: the cost at the robot's cell drops below
 *    INSCRIBED_INFLATED_OBSTACLE (253). This is the upstream condition
 *    the planner needs to seed a path; works with the current BT
 *    structure where the planner only re-fires after the recovery branch
 *    returns SUCCESS.
 *
 * Reuses the DriveOnHeading action type so no custom msg package is
 * needed. In the BT, invoke with server_name="gradient_escape".
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
  double escape_speed_;          // m/s, magnitude commanded along escape vector
  double max_search_radius_m_;   // bounded search radius for nearest lethal cell
  double timeout_s_;             // seconds
  std::string plan_topic_;       // planner output topic, default "/plan"

  rclcpp::Time start_time_;
  DriveOnHeadingAction::Feedback::SharedPtr feedback_;

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  std::mutex plan_mutex_;
  nav_msgs::msg::Path::SharedPtr latest_plan_;
};

}  // namespace gradient_escape

#endif  // GRADIENT_ESCAPE__GRADIENT_ESCAPE_HPP_
