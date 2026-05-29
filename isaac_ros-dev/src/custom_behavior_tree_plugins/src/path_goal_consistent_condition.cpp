#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <string>

namespace autonav_bt {

class PathGoalConsistent : public BT::ActionNodeBase
{
public:
  PathGoalConsistent(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("action_goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("planner_goal"),
      BT::InputPort<double>("goal_tolerance_m", 0.75, "Allowed path-end to planner-goal distance"),
      BT::InputPort<double>("stamp_tolerance_s", 0.02, "Allowed path stamp age before the action goal"),
      BT::InputPort<double>("stale_timeout_s", 1.50, "How long to wait for a fresh matching path"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto status = evaluate();
    if (status == BT::NodeStatus::SUCCESS) {
      waiting_ = false;
      return status;
    }

    const auto now = node_->now();
    if (!waiting_) {
      waiting_ = true;
      wait_started_ = now;
    }

    double stale_timeout_s = 1.50;
    getInput("stale_timeout_s", stale_timeout_s);
    if (stale_timeout_s > 0.0 && (now - wait_started_).seconds() > stale_timeout_s) {
      RCLCPP_WARN(
        node_->get_logger(),
        "PathGoalConsistent: stale path did not refresh within %.2fs: %s",
        stale_timeout_s, last_reason_.c_str());
      waiting_ = false;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "PathGoalConsistent: waiting for planner path to match current goal: %s",
      last_reason_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    waiting_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  BT::NodeStatus evaluate()
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
      last_reason_ = "path is empty";
      return BT::NodeStatus::RUNNING;
    }

    geometry_msgs::msg::PoseStamped action_goal;
    if (!getInput("action_goal", action_goal)) {
      last_reason_ = "action_goal missing";
      return BT::NodeStatus::RUNNING;
    }

    geometry_msgs::msg::PoseStamped planner_goal;
    if (!getInput("planner_goal", planner_goal)) {
      planner_goal = action_goal;
    }

    double goal_tolerance_m = 0.75;
    double stamp_tolerance_s = 0.02;
    getInput("goal_tolerance_m", goal_tolerance_m);
    getInput("stamp_tolerance_s", stamp_tolerance_s);

    const double action_stamp_s = stampToSeconds(action_goal.header.stamp);
    const double path_stamp_s = stampToSeconds(path.header.stamp);
    if (action_stamp_s > 0.0 && path_stamp_s > 0.0 &&
        path_stamp_s + stamp_tolerance_s < action_stamp_s)
    {
      last_reason_ =
        "path stamp " + std::to_string(path_stamp_s) +
        " predates action goal stamp " + std::to_string(action_stamp_s);
      return BT::NodeStatus::RUNNING;
    }

    const auto & end = path.poses.back().pose.position;
    const auto & goal = planner_goal.pose.position;
    const double d = std::hypot(end.x - goal.x, end.y - goal.y);
    if (!std::isfinite(d) || d > goal_tolerance_m) {
      last_reason_ =
        "path endpoint (" + std::to_string(end.x) + ", " + std::to_string(end.y) +
        ") is " + std::to_string(d) + "m from planner goal (" +
        std::to_string(goal.x) + ", " + std::to_string(goal.y) + ")";
      return BT::NodeStatus::RUNNING;
    }

    last_reason_.clear();
    return BT::NodeStatus::SUCCESS;
  }

  static double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
  }

  rclcpp::Node::SharedPtr node_;
  bool waiting_ = false;
  rclcpp::Time wait_started_{0, 0, RCL_ROS_TIME};
  std::string last_reason_;
};

}  // namespace autonav_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autonav_bt::PathGoalConsistent>("PathGoalConsistent");
}
