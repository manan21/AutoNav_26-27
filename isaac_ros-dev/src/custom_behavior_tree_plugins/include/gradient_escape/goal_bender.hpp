#ifndef GRADIENT_ESCAPE__GOAL_BENDER_HPP_
#define GRADIENT_ESCAPE__GOAL_BENDER_HPP_

#include <atomic>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace gradient_escape
{

// Emits a forward intermediate goal when the previously-planned path
// loops behind the robot. The bend fires when:
//   * goal AND path are both behind the robot                  (always), OR
//   * path is behind the robot AND the breadcrumb buffer is
//     empty (i.e. the breadcrumb-reverse recovery has nothing
//     left to spend, so we must turn the robot around).
//
// In every other case (goal-in-front-of-robot, path-in-front, or
// path-behind while goal-in-front with breadcrumbs available) the
// input_goal is passed through unchanged — the breadcrumb-reverse
// behavior takes over the "goal-front / path-behind" case.
class GoalBender : public BT::SyncActionNode
{
public:
  GoalBender(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal"),
      BT::InputPort<double>("bend_distance", 1.5, "Forward intermediate-goal distance (m)"),
      BT::InputPort<double>("angle_threshold", 1.57, "Behind-robot angle threshold (rad)"),
      BT::InputPort<double>("bend_angle", 1.05, "Forward-bend angle offset (rad)"),
      BT::InputPort<nav_msgs::msg::Path>("previous_path", "Last path from ComputePathToPose; enables path-direction trigger"),
      // Pure-pursuit lookahead distance (matches IsForwardBlocked).
      // > 0 selects closest-point + scan-forward (sim-validated);
      // == 0 falls back to the legacy fixed `path_lookahead_index`.
      BT::InputPort<double>("min_lookahead_m", 0.6,
        "Pure-pursuit lookahead distance (m). 0 = use path_lookahead_index."),
      BT::InputPort<int>("path_lookahead_index", 5,
        "[LEGACY] Fixed waypoint index. Only used when min_lookahead_m <= 0."),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr breadcrumb_sub_;
  std::atomic<bool> breadcrumb_buffer_empty_{true};
};

}  // namespace gradient_escape

#endif  // GRADIENT_ESCAPE__GOAL_BENDER_HPP_
