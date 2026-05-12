#ifndef GRADIENT_ESCAPE__GOAL_BENDER_HPP_
#define GRADIENT_ESCAPE__GOAL_BENDER_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace gradient_escape
{

// Bends the goal toward a committed queue of intermediate waypoints
// when the goal or the previous path lies behind the robot's heading.
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
      BT::InputPort<int>("queue_size", 5, "Mini-waypoints sampled from current pose to U-tip"),
      BT::InputPort<double>("reach_radius", 0.5, "Robot-to-current-waypoint distance to advance the queue (m)"),
      BT::InputPort<double>("goal_change_threshold", 2.0, "Real-goal move (m) that rebuilds the queue"),
      BT::InputPort<double>("tip_change_threshold", 1.0, "Current-path U-tip drift (m) that invalidates the queue"),
    };
  }

  BT::NodeStatus tick() override;

private:
  // Persists across ticks (BT.CPP keeps a single node instance).
  // Prevents the per-tick re-bend that was causing the robot to
  // oscillate between "follow the U" and "replan the U".
  std::vector<geometry_msgs::msg::PoseStamped> committed_queue_;
  size_t committed_idx_ = 0;
  geometry_msgs::msg::PoseStamped committed_real_goal_;
  // U-tip position at queue build — if the live path's tip drifts
  // away from this, the queue is following a stale U-shape and gets
  // rebuilt or released.
  double committed_tip_x_ = 0.0;
  double committed_tip_y_ = 0.0;
  bool has_commitment_ = false;
};

}  // namespace gradient_escape

#endif  // GRADIENT_ESCAPE__GOAL_BENDER_HPP_
