#ifndef BREADCRUMB__BREADCRUMB_BUFFER_HPP_
#define BREADCRUMB__BREADCRUMB_BUFFER_HPP_

#include <deque>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace breadcrumb
{

// Drops a breadcrumb pose every `stride_m` of forward travel, keeping a
// ring buffer of the last `buffer_size` poses in the odom frame.
// Republishes the current tail on /breadcrumb_tail.
//
// During reverse motion, when the robot pose returns within
// `consume_tolerance_m` of the most recent breadcrumb, that breadcrumb
// is popped — so as breadcrumb_reverse drives the robot backward along
// the tail, the buffer naturally drains from the head.
class BreadcrumbBuffer : public rclcpp::Node
{
public:
  BreadcrumbBuffer();

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishTail();
  static double dist2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tail_pub_;
  rclcpp::TimerBase::SharedPtr tail_timer_;

  double stride_m_;
  size_t buffer_size_;
  double min_forward_vx_;
  double consume_tolerance_m_;
  std::string odom_frame_;
  std::string odom_topic_;

  std::deque<geometry_msgs::msg::PoseStamped> tail_;
  geometry_msgs::msg::PoseStamped last_drop_pose_;
  bool have_last_drop_{false};
};

}  // namespace breadcrumb

#endif  // BREADCRUMB__BREADCRUMB_BUFFER_HPP_
