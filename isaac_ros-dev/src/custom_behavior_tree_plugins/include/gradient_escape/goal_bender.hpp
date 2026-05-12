#ifndef GRADIENT_ESCAPE__GOAL_BENDER_HPP_
#define GRADIENT_ESCAPE__GOAL_BENDER_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace gradient_escape
{

// Bends the goal toward an intermediate waypoint when the goal or
// the previous path lies behind the robot's heading.
class GoalBender : public BT::SyncActionNode
{
public:
  GoalBender(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal"),
      BT::InputPort<double>("bend_distance", 1.5, "Fallback fixed-arc bend distance (m)"),
      BT::InputPort<double>("angle_threshold", 1.57, "Behind-robot angle threshold (rad)"),
      BT::InputPort<double>("bend_angle", 1.05, "Fallback fixed-arc offset (rad)"),
      BT::InputPort<nav_msgs::msg::Path>("previous_path", "Last path from ComputePathToPose; enables path-direction trigger"),
      BT::InputPort<int>("path_lookahead_index", 5, "Waypoint index used to test path direction"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace gradient_escape

#endif  // GRADIENT_ESCAPE__GOAL_BENDER_HPP_
