#ifndef AUTONAV_HYBRID_PLANNER__LOCAL_THEN_STRAIGHT_PLANNER_HPP_
#define AUTONAV_HYBRID_PLANNER__LOCAL_THEN_STRAIGHT_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace autonav_hybrid_planner
{

class LocalThenStraightPlanner : public nav2_core::GlobalPlanner
{
public:
  LocalThenStraightPlanner();
  ~LocalThenStraightPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  geometry_msgs::msg::PoseStamped makeHandoffGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double distance) const;

  nav_msgs::msg::Path appendStraightSegment(
    nav_msgs::msg::Path near_path,
    const geometry_msgs::msg::PoseStamped & goal) const;

  bool straightSegmentIsAllowed(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & goal) const;

  geometry_msgs::msg::PoseStamped choosePlanningStart(
    const geometry_msgs::msg::PoseStamped & start) const;

  bool startCellIsBlocked(
    const geometry_msgs::msg::PoseStamped & start) const;

  bool costIsBlocked(unsigned char cost) const;

  // Footprint-aware start validity. SmacPlannerLattice rejects a start when any
  // cell under the oriented robot footprint is lethal, not just the center
  // cell, so the start-relaxation must use the same footprint test or it hands
  // Smac a center-clear-but-footprint-lethal pose and planning still aborts.
  bool footprintIsBlocked(double wx, double wy, double yaw) const;

  bool lineIsBlocked(double x0, double y0, double x1, double y1) const;

  // Cells covered by the robot footprint at a world pose+yaw, used to clear
  // stale cost the body is sitting on before planning so a start whose
  // footprint touches cost can never abort the plan.
  std::vector<unsigned int> footprintCellIndices(double wx, double wy, double yaw) const;

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);

  nav_msgs::msg::Path prependActualStart(
    nav_msgs::msg::Path path,
    const geometry_msgs::msg::PoseStamped & actual_start,
    const geometry_msgs::msg::PoseStamped & planning_start) const;

  static double yawBetween(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to);

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> near_planner_loader_;
  nav2_core::GlobalPlanner::Ptr near_planner_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_;

  std::string name_;
  std::string global_frame_;
  std::string near_planner_plugin_;
  std::string near_planner_name_;
  std::string near_planner_full_name_;

  double local_horizon_m_;
  double close_goal_distance_m_;
  double far_path_spacing_m_;
  double start_search_radius_m_;
  unsigned char start_blocked_cost_threshold_;
  bool allow_unknown_start_;
  bool relax_blocked_start_;
  bool far_collision_check_;
  bool configured_;
};

}  // namespace autonav_hybrid_planner

#endif  // AUTONAV_HYBRID_PLANNER__LOCAL_THEN_STRAIGHT_PLANNER_HPP_
