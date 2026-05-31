#include "breadcrumb/breadcrumb_buffer.hpp"

#include <cmath>

namespace breadcrumb
{

BreadcrumbBuffer::BreadcrumbBuffer()
: rclcpp::Node("breadcrumb_buffer")
{
  stride_m_ = declare_parameter<double>("stride_m", 0.10);
  buffer_size_ = static_cast<size_t>(declare_parameter<int>("buffer_size", 10));
  min_forward_vx_ = declare_parameter<double>("min_forward_vx", 0.05);
  consume_tolerance_m_ = declare_parameter<double>("consume_tolerance_m", 0.05);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  // Source of truth for the odom frame: the local EKF output, which
  // also broadcasts odom→base_link TF (see slam.launch.py). Using raw
  // /odom (wheel odometry) would drift relative to the TF-resolved
  // pose breadcrumb_reverse looks up, causing crumb drop/consume
  // mismatches over time.
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/local_ekf/odom");

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10),
    std::bind(&BreadcrumbBuffer::onOdom, this, std::placeholders::_1));

  // Transient-local so any node spinning up later (GoalBender,
  // BreadcrumbReverse) immediately sees the current tail state without
  // waiting for the next publish tick.
  tail_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/breadcrumb_tail", rclcpp::QoS(1).transient_local().reliable());

  tail_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&BreadcrumbBuffer::publishTail, this));

  RCLCPP_INFO(get_logger(),
    "BreadcrumbBuffer: stride=%.2fm size=%zu min_vx=%.2fm/s consume_tol=%.2fm",
    stride_m_, buffer_size_, min_forward_vx_, consume_tolerance_m_);
}

double BreadcrumbBuffer::dist2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

void BreadcrumbBuffer::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped current;
  current.header.stamp = msg->header.stamp;
  current.header.frame_id = odom_frame_;
  current.pose = msg->pose.pose;

  const double vx = msg->twist.twist.linear.x;

  // Drop while moving forward. Time-based dropping would over-sample at
  // low speed; gating on (vx > min) AND (Δd > stride) keeps the tail
  // tied to actual ground covered.
  if (vx > min_forward_vx_) {
    if (!have_last_drop_) {
      tail_.push_back(current);
      last_drop_pose_ = current;
      have_last_drop_ = true;
    } else {
      const double d = dist2D(current.pose.position, last_drop_pose_.pose.position);
      if (d >= stride_m_) {
        tail_.push_back(current);
        last_drop_pose_ = current;
        while (tail_.size() > buffer_size_) {
          tail_.pop_front();
        }
      }
    }
    return;
  }

  // Pop on close approach to the most recent crumb. The forward-drop
  // branch above returned out, so any execution reaching here is
  // non-forward (reversing, stopped, or coasting forward below
  // min_forward_vx_) — i.e. a state where, if we're standing on a
  // crumb, it's stale and should be consumed. Gating only on
  // (vx < -min_forward_vx_) missed cases where breadcrumb_reverse's
  // align_factor drove linear.x near zero just before arrival (target
  // off-axis), leaving the crumb un-popped and re-targeted on the
  // next BreadcrumbReverse invocation.
  if (!tail_.empty()) {
    const auto & latest = tail_.back();
    const double d = dist2D(current.pose.position, latest.pose.position);
    if (d <= consume_tolerance_m_) {
      tail_.pop_back();
      // Reset last_drop so the next forward run begins a fresh stride
      // count from wherever the robot currently is.
      have_last_drop_ = false;
    }
  }
}

void BreadcrumbBuffer::publishTail()
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = odom_frame_;
  msg.poses.assign(tail_.begin(), tail_.end());
  tail_pub_->publish(msg);
}

}  // namespace breadcrumb

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<breadcrumb::BreadcrumbBuffer>());
  rclcpp::shutdown();
  return 0;
}
