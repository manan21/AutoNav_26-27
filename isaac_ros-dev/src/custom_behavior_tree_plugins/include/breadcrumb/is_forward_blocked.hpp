#ifndef BREADCRUMB__IS_FORWARD_BLOCKED_HPP_
#define BREADCRUMB__IS_FORWARD_BLOCKED_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace breadcrumb
{

// BT condition: SUCCESS when the planned {path} loops behind the robot
// while the actual {goal} is in front. This is the case a forward-only
// controller should not try to follow — the breadcrumb-reverse
// behavior should take over.
//
// Returns FAILURE in every other configuration (path in front, no path
// yet, or goal also behind — that last one belongs to GoalBender, not
// here).
class IsForwardBlocked : public BT::ConditionNode
{
public:
  IsForwardBlocked(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned path from ComputePathToPose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal"),
      BT::InputPort<double>("angle_threshold", 1.57, "Behind-robot angle threshold (rad)"),
      // Lookahead distance (metres) for the pure-pursuit-style first-
      // forward-waypoint check. When > 0, the closest-point + scan-
      // forward variant is used (recommended — matches the sim). When
      // == 0, falls back to the legacy fixed `path_lookahead_index`.
      BT::InputPort<double>("min_lookahead_m", 0.6,
        "Pure-pursuit lookahead distance (m). 0 = use path_lookahead_index."),
      BT::InputPort<int>("path_lookahead_index", 5,
        "[LEGACY] Fixed waypoint index. Only used when min_lookahead_m <= 0."),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace breadcrumb

#endif  // BREADCRUMB__IS_FORWARD_BLOCKED_HPP_
