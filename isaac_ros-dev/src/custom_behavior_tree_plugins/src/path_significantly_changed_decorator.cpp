#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

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
    if (!node_->has_parameter("path_significantly_changed.max_point_delta_m")) {
      node_->declare_parameter("path_significantly_changed.max_point_delta_m", 0.30);
    }
    if (!node_->has_parameter("path_significantly_changed.start_delta_threshold_m")) {
      node_->declare_parameter("path_significantly_changed.start_delta_threshold_m", 0.25);
    }
    if (!node_->has_parameter("path_significantly_changed.length_delta_threshold_m")) {
      node_->declare_parameter("path_significantly_changed.length_delta_threshold_m", 0.50);
    }
    if (!node_->has_parameter("path_significantly_changed.force_update_period_s")) {
      node_->declare_parameter("path_significantly_changed.force_update_period_s", 0.5);
    }
    if (!node_->has_parameter("path_significantly_changed.empty_path_hold_period_s")) {
      node_->declare_parameter("path_significantly_changed.empty_path_hold_period_s", 1.0);
    }
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<nav_msgs::msg::Path>("filtered_path"),
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
    const double max_point_delta = node_->get_parameter(
      "path_significantly_changed.max_point_delta_m").as_double();
    const double start_delta_threshold = node_->get_parameter(
      "path_significantly_changed.start_delta_threshold_m").as_double();
    const double length_delta_threshold = node_->get_parameter(
      "path_significantly_changed.length_delta_threshold_m").as_double();
    const double force_update_period = node_->get_parameter(
      "path_significantly_changed.force_update_period_s").as_double();
    const double empty_path_hold_period = node_->get_parameter(
      "path_significantly_changed.empty_path_hold_period_s").as_double();

    const auto last_status = child_node_->status();

    if (new_path.poses.empty()) {
      if (has_filtered_path_ && child_started_ &&
          last_status == BT::NodeStatus::RUNNING && empty_path_hold_period > 0.0)
      {
        const auto now = node_->now();
        if (!has_empty_path_hold_time_) {
          empty_path_start_time_ = now;
          has_empty_path_hold_time_ = true;
        }

        const double held_for = (now - empty_path_start_time_).seconds();
        if (held_for <= empty_path_hold_period) {
          setOutput("filtered_path", filtered_path_);
          return child_node_->executeTick();
        }
      }

      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathSignificantlyChanged: refusing to tick FollowPath with an empty path");
      resetChildState();
      return BT::NodeStatus::FAILURE;
    }

    has_empty_path_hold_time_ = false;

    const bool force_update =
      has_last_update_time_ && force_update_period > 0.0 &&
      (node_->now() - last_update_time_).seconds() >= force_update_period;

    const bool significantly_changed = force_update || pathDiffers(
      new_path, filtered_path_, rms_threshold, max_point_delta,
      start_delta_threshold, length_delta_threshold, n_compare);

    if (significantly_changed || !child_started_ || !has_filtered_path_ ||
        last_status == BT::NodeStatus::IDLE) {
      // Only publish a new child-visible path when the raw planner path
      // changed enough to justify a FollowPath goal update. The child is
      // still ticked every BT cycle below so Nav2's action node can spin
      // feedback/result callbacks; it just sees this stable filtered path
      // while same-looking replans arrive.
      filtered_path_ = new_path;
      has_filtered_path_ = true;
      child_started_ = true;
      last_update_time_ = node_->now();
      has_last_update_time_ = true;
    }

    setOutput("filtered_path", filtered_path_);
    return child_node_->executeTick();
  }

  // CRITICAL: override halt() so a recovery-triggered halt clears the
  // started flag. Without this, after the ReactiveFallback fires (e.g.
  // BackUp + ClearCostmaps + Wait) and returns to the main pipeline,
  // the decorator would see the same path on the blackboard and refuse
  // to re-tick the child — FollowPath would never get a fresh action
  // goal and the robot would stop.
  void halt() override {
    resetChildState();
    BT::DecoratorNode::halt();
  }

private:
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path filtered_path_;
  bool child_started_ = false;
  bool has_filtered_path_ = false;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  bool has_last_update_time_ = false;
  rclcpp::Time empty_path_start_time_{0, 0, RCL_ROS_TIME};
  bool has_empty_path_hold_time_ = false;

  void resetChildState() {
    if (child_node_->status() != BT::NodeStatus::IDLE) {
      child_node_->halt();
    }
    child_started_ = false;
    has_filtered_path_ = false;
    has_last_update_time_ = false;
    has_empty_path_hold_time_ = false;
    filtered_path_ = nav_msgs::msg::Path{};
  }

  static bool pathDiffers(const nav_msgs::msg::Path & a,
                          const nav_msgs::msg::Path & b,
                          double rms_threshold_m,
                          double max_point_delta_m,
                          double start_delta_threshold_m,
                          double length_delta_threshold_m,
                          int n_compare) {
    if (a.poses.empty() || b.poses.empty()) return true;

    // Goal-endpoint change check — fires when a NEW NavigateToPose goal
    // arrives. Without this, the RMS-over-first-N-poses check below can
    // miss a new goal whose path happens to start in the same direction
    // as the last path (most common when the robot is sitting still
    // near the previous goal: front poses byte-identical, RMS over the
    // first 50 cm of two different goals' paths can fall under
    // rms_threshold_m, decorator returns RUNNING without ticking
    // FollowPath, and the robot only moves on the second goal-send
    // because that forces a tree halt-reset). Endpoint comparison is
    // unambiguous: a new goal moves the path's last pose, period.
    // Threshold is generous (3× the planner's typical xy_goal_tolerance)
    // so a one-tick goal-pose jitter doesn't fire false positives.
    constexpr double GOAL_ENDPOINT_DELTA_M = 0.30;
    const auto & a_end = a.poses.back().pose.position;
    const auto & b_end = b.poses.back().pose.position;
    const double dxe = a_end.x - b_end.x;
    const double dye = a_end.y - b_end.y;
    if (std::sqrt(dxe*dxe + dye*dye) > GOAL_ENDPOINT_DELTA_M) {
      return true;
    }

    // Start-pose displacement should not fire on every planner tick. The
    // controller can prune progress along the same path; this is only for
    // large jumps such as a relocalization, recovery, or route reset.
    const double sdx =
      a.poses.front().pose.position.x - b.poses.front().pose.position.x;
    const double sdy =
      a.poses.front().pose.position.y - b.poses.front().pose.position.y;
    if (std::sqrt(sdx*sdx + sdy*sdy) > start_delta_threshold_m) {
      return true;
    }

    const double len_a = pathLength(a);
    const double len_b = pathLength(b);
    if (std::abs(len_a - len_b) > length_delta_threshold_m) {
      return true;
    }

    const size_t n = std::max<size_t>(2, static_cast<size_t>(n_compare));
    double sum_sq = 0.0;
    double max_delta = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double fraction = static_cast<double>(i) / static_cast<double>(n - 1);
      const auto pa = pointAtFraction(a, len_a, fraction);
      const auto pb = pointAtFraction(b, len_b, fraction);
      if (!std::isfinite(pa.first) || !std::isfinite(pb.first)) {
        return true;
      }
      const double dx = pa.first - pb.first;
      const double dy = pa.second - pb.second;
      const double d2 = dx * dx + dy * dy;
      sum_sq += d2;
      max_delta = std::max(max_delta, std::sqrt(d2));
    }
    const double rms = std::sqrt(sum_sq / static_cast<double>(n));
    return rms > rms_threshold_m || max_delta > max_point_delta_m;
  }

  static double pathLength(const nav_msgs::msg::Path & path) {
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      length += std::hypot(b.x - a.x, b.y - a.y);
    }
    return length;
  }

  static std::pair<double, double> pointAtFraction(
    const nav_msgs::msg::Path & path,
    double path_length,
    double fraction)
  {
    if (path.poses.empty()) {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      return {nan, nan};
    }
    if (path.poses.size() == 1 || path_length <= 1e-6) {
      const auto & p = path.poses.front().pose.position;
      return {p.x, p.y};
    }

    const double target = std::clamp(fraction, 0.0, 1.0) * path_length;
    double traversed = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      const double segment = std::hypot(b.x - a.x, b.y - a.y);
      if (segment <= 1e-6) {
        continue;
      }
      if (traversed + segment >= target) {
        const double ratio = (target - traversed) / segment;
        return {
          a.x + ratio * (b.x - a.x),
          a.y + ratio * (b.y - a.y),
        };
      }
      traversed += segment;
    }
    const auto & p = path.poses.back().pose.position;
    return {p.x, p.y};
  }
};

}  // namespace autonav_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonav_bt::PathSignificantlyChanged>(
    "PathSignificantlyChanged");
}
