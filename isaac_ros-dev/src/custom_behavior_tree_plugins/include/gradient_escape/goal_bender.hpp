#ifndef GRADIENT_ESCAPE__GOAL_BENDER_HPP_
#define GRADIENT_ESCAPE__GOAL_BENDER_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace gradient_escape
{

/**
 * @brief BT node that bends the goal when it's behind the robot.
 *
 * If the goal is behind the robot (relative angle exceeds a threshold),
 * this node computes an intermediate waypoint ahead of the robot and
 * offset toward the goal's side.  The robot naturally arcs toward the
 * real goal.  If the goal is already in front, it passes through
 * unchanged.
 *
 * Ports:
 *   input_goal   — the real goal from NavigateToPose
 *   output_goal  — the (possibly bent) goal for ComputePathToPose
 */
class GoalBender : public BT::SyncActionNode
{
public:
  GoalBender(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal"),
      BT::InputPort<double>("bend_distance", 1.5,
        "Distance (m) ahead for the intermediate waypoint"),
      BT::InputPort<double>("angle_threshold", 1.57,
        "Angle (rad) beyond which the goal is considered 'behind'"),
      BT::InputPort<double>("bend_angle", 1.05,
        "Offset angle (rad, ~60 deg) toward the goal side"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace gradient_escape

#endif  // GRADIENT_ESCAPE__GOAL_BENDER_HPP_
