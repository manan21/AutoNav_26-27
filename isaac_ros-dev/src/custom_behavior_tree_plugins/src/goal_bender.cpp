#include "gradient_escape/goal_bender.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace gradient_escape
{

namespace
{
// Normalize an angle to [-pi, pi].
inline double wrap_pi(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

GoalBender::GoalBender(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GoalBender::tick()
{
  // --- read ports ---
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("input_goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }

  double bend_dist = 1.5;
  double angle_thresh = 1.57;   // ~90 deg
  double bend_angle = 1.05;     // ~60 deg
  int    path_lookahead_idx = 5;
  getInput("bend_distance", bend_dist);
  getInput("angle_threshold", angle_thresh);
  getInput("bend_angle", bend_angle);
  getInput("path_lookahead_index", path_lookahead_idx);

  // --- get robot pose from blackboard TF ---
  auto tf_buffer =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Can't get pose — pass goal through unchanged
    setOutput("output_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double ryaw = tf2::getYaw(tf.transform.rotation);

  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  const double rel_goal = wrap_pi(std::atan2(gy - ry, gx - rx) - ryaw);
  const bool goal_behind = std::abs(rel_goal) > angle_thresh;

  nav_msgs::msg::Path prev_path;
  const bool have_path =
    getInput("previous_path", prev_path) && prev_path.poses.size() >= 2;
  bool path_behind = false;
  if (have_path) {
    const int idx = std::min(
      static_cast<int>(prev_path.poses.size()) - 1,
      std::max(1, path_lookahead_idx));
    const double lx = prev_path.poses[idx].pose.position.x;
    const double ly = prev_path.poses[idx].pose.position.y;
    const double rel_look =
      wrap_pi(std::atan2(ly - ry, lx - rx) - ryaw);
    path_behind = std::abs(rel_look) > angle_thresh;
  }

  if (!goal_behind && !path_behind) {
    // Trap cleared — release any previous commitment and pass through.
    has_commitment_ = false;
    committed_queue_.clear();
    setOutput("output_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

  int queue_size = 5;
  double reach_radius = 0.5;
  double goal_change_thresh = 2.0;
  double tip_change_thresh = 1.0;
  getInput("queue_size", queue_size);
  getInput("reach_radius", reach_radius);
  getInput("goal_change_threshold", goal_change_thresh);
  getInput("tip_change_threshold", tip_change_thresh);

  // Compute the current live path's U-tip (waypoint furthest from
  // robot). If we have a commitment AND the live tip has drifted
  // away from where it was when we built the queue, the planner is
  // proposing a different U-shape — abandon the stale queue.
  size_t live_tip = 0;
  double live_tip_dist = 0.0;
  if (have_path) {
    for (size_t i = 1; i < prev_path.poses.size(); ++i) {
      const double dx = prev_path.poses[i].pose.position.x - rx;
      const double dy = prev_path.poses[i].pose.position.y - ry;
      const double d = std::hypot(dx, dy);
      if (d > live_tip_dist) {
        live_tip_dist = d;
        live_tip = i;
      }
    }
  }

  // If the real goal moved a lot, the trap has changed shape — rebuild.
  if (has_commitment_) {
    const double dgx = gx - committed_real_goal_.pose.position.x;
    const double dgy = gy - committed_real_goal_.pose.position.y;
    if (std::hypot(dgx, dgy) > goal_change_thresh) {
      has_commitment_ = false;
      committed_queue_.clear();
    }
  }

  // If the live path's tip has drifted, the queue is stale.
  if (has_commitment_ && have_path) {
    const double dtx =
      prev_path.poses[live_tip].pose.position.x - committed_tip_x_;
    const double dty =
      prev_path.poses[live_tip].pose.position.y - committed_tip_y_;
    if (std::hypot(dtx, dty) > tip_change_thresh) {
      has_commitment_ = false;
      committed_queue_.clear();
    }
  }

  // Advance the queue when the robot reaches the current waypoint.
  if (has_commitment_ && committed_idx_ < committed_queue_.size()) {
    const auto & wp = committed_queue_[committed_idx_].pose.position;
    if (std::hypot(wp.x - rx, wp.y - ry) < reach_radius) {
      committed_idx_++;
    }
    if (committed_idx_ >= committed_queue_.size()) {
      has_commitment_ = false;
      committed_queue_.clear();
    }
  }

  if (!has_commitment_ && have_path && live_tip >= 1 && queue_size > 0) {
    committed_queue_.clear();
    for (int n = 1; n <= queue_size; ++n) {
      const size_t s = (live_tip * static_cast<size_t>(n)) /
                       static_cast<size_t>(queue_size);
      if (s >= prev_path.poses.size()) continue;
      committed_queue_.push_back(prev_path.poses[s]);
    }
    committed_idx_ = 0;
    committed_real_goal_ = goal;
    committed_tip_x_ = prev_path.poses[live_tip].pose.position.x;
    committed_tip_y_ = prev_path.poses[live_tip].pose.position.y;
    has_commitment_ = !committed_queue_.empty();
  }

  geometry_msgs::msg::PoseStamped bent;
  bent.header = goal.header;
  bent.header.stamp = node->get_clock()->now();
  bent.pose.position.z = 0.0;

  if (has_commitment_ && committed_idx_ < committed_queue_.size()) {
    bent.pose.position.x =
      committed_queue_[committed_idx_].pose.position.x;
    bent.pose.position.y =
      committed_queue_[committed_idx_].pose.position.y;
  } else {
    // No path / no queue — fall back to the legacy fixed-arc bend.
    const double offset = (rel_goal >= 0.0) ? bend_angle : -bend_angle;
    const double heading = ryaw + offset;
    bent.pose.position.x = rx + bend_dist * std::cos(heading);
    bent.pose.position.y = ry + bend_dist * std::sin(heading);
  }

  const double to_real = std::atan2(
    gy - bent.pose.position.y, gx - bent.pose.position.x);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, to_real);
  bent.pose.orientation = tf2::toMsg(q);

  setOutput("output_goal", bent);

  RCLCPP_INFO(node->get_logger(),
    "GoalBender: trigger=%s%s%s -> wp[%zu/%zu] (%.1f, %.1f)",
    goal_behind ? "goal-behind" : "",
    (goal_behind && path_behind) ? "+" : "",
    path_behind ? "path-behind" : "",
    committed_idx_ + 1, committed_queue_.size(),
    bent.pose.position.x, bent.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace gradient_escape

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gradient_escape::GoalBender>("GoalBender");
}
