#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>

namespace autonav_bt {

class PathSignificantlyChanged : public BT::DecoratorNode
{
public:
  PathSignificantlyChanged(const std::string & name,
                           const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    // Tuning lives in bt_navigator's ROS params (see nav2_paramsv2.yaml),
    // not in BT XML input ports — so the values can be edited in YAML or
    // changed live with `ros2 param set /bt_navigator ...` without rebuilding
    // or re-loading the BT.
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    if (!node_->has_parameter("path_significantly_changed.rms_threshold_m")) {
      node_->declare_parameter("path_significantly_changed.rms_threshold_m", 0.10);
    }
    if (!node_->has_parameter("path_significantly_changed.compare_n_poses")) {
      node_->declare_parameter("path_significantly_changed.compare_n_poses", 10);
    }
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
    };
  }

  BT::NodeStatus tick() override {
    nav_msgs::msg::Path new_path;
    if (!getInput("path", new_path)) {
      return BT::NodeStatus::FAILURE;
    }

    // Re-read every tick so live `ros2 param set` takes effect immediately.
    const double rms_threshold = node_->get_parameter(
      "path_significantly_changed.rms_threshold_m").as_double();
    const int n_compare = node_->get_parameter(
      "path_significantly_changed.compare_n_poses").as_int();

    const bool significantly_changed = pathDiffers(
      new_path, last_path_, rms_threshold, n_compare);

    if (significantly_changed || !child_started_) {
      // Path changed enough OR child hasn't run yet — tick the child
      // (FollowPath), which on a fresh path will cancel-restart its
      // action goal. This is the ONLY place we let cancel-restart fire.
      last_path_ = new_path;
      child_started_ = true;
      return child_node_->executeTick();
    }

    // Path didn't materially change — return RUNNING so the parent
    // PipelineSequence keeps thinking the child is alive, but DO NOT
    // tick the child. FollowPath's action goal stays exactly where
    // it was; the controller_server keeps following the existing path
    // uninterrupted.
    return BT::NodeStatus::RUNNING;
  }

  // CRITICAL: override halt() so a recovery-triggered halt clears the
  // started flag. Without this, after the ReactiveFallback fires (e.g.
  // BackUp + ClearCostmaps + Wait) and returns to the main pipeline,
  // the decorator would see the same path on the blackboard and refuse
  // to re-tick the child — FollowPath would never get a fresh action
  // goal and the robot would stop.
  void halt() override {
    child_started_ = false;
    last_path_ = nav_msgs::msg::Path{};
    BT::DecoratorNode::halt();
  }

private:
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path last_path_;
  bool child_started_ = false;

  static bool pathDiffers(const nav_msgs::msg::Path & a,
                          const nav_msgs::msg::Path & b,
                          double rms_threshold_m,
                          int n_compare) {
    if (a.poses.empty() || b.poses.empty()) return true;
    if (a.poses.front().pose.position.x != b.poses.front().pose.position.x
     || a.poses.front().pose.position.y != b.poses.front().pose.position.y) {
      // First pose moved — definitely a new plan.
      return true;
    }
    const size_t n = std::min({static_cast<size_t>(n_compare),
                               a.poses.size(), b.poses.size()});
    double sum_sq = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double dx = a.poses[i].pose.position.x - b.poses[i].pose.position.x;
      const double dy = a.poses[i].pose.position.y - b.poses[i].pose.position.y;
      sum_sq += dx*dx + dy*dy;
    }
    const double rms = std::sqrt(sum_sq / static_cast<double>(n));
    return rms > rms_threshold_m;
  }
};

}  // namespace autonav_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonav_bt::PathSignificantlyChanged>(
    "PathSignificantlyChanged");
}
