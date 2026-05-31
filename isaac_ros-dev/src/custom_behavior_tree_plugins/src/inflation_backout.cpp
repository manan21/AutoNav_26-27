#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/robot_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace inflation_backout
{

using BackUpAction = nav2_msgs::action::BackUp;
using Status = nav2_behaviors::Status;

class InflationBackout : public nav2_behaviors::TimedBehavior<BackUpAction>
{
public:
  InflationBackout()
  : nav2_behaviors::TimedBehavior<BackUpAction>(),
    target_distance_(0.20),
    reverse_speed_(-0.05)
  {
  }

  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override
  {
    target_distance_ = std::max(0.01, std::abs(command->target.x));
    reverse_speed_ = -std::max(0.01, std::abs(static_cast<double>(command->speed)));
    end_time_ = clock_->now() + command->time_allowance;

    if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "InflationBackout: initial robot pose is not available");
      return Status::FAILED;
    }

    feedback_ = std::make_shared<BackUpAction::Feedback>();
    RCLCPP_WARN(
      logger_,
      "InflationBackout: reversing %.2f m at %.2f m/s without current-footprint "
      "collision veto",
      target_distance_, reverse_speed_);

    return Status::SUCCEEDED;
  }

  Status onCycleUpdate() override
  {
    const rclcpp::Duration time_remaining = end_time_ - clock_->now();
    if (time_remaining.seconds() < 0.0) {
      stopRobot();
      RCLCPP_WARN(logger_, "InflationBackout: timed out before backing out");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      stopRobot();
      RCLCPP_ERROR(logger_, "InflationBackout: current robot pose is not available");
      return Status::FAILED;
    }

    const double dx = initial_pose_.pose.position.x - current_pose.pose.position.x;
    const double dy = initial_pose_.pose.position.y - current_pose.pose.position.y;
    const double distance = std::hypot(dx, dy);

    feedback_->distance_traveled = static_cast<float>(distance);
    action_server_->publish_feedback(feedback_);

    if (distance >= target_distance_) {
      stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = reverse_speed_;
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;
    vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

private:
  geometry_msgs::msg::PoseStamped initial_pose_;
  rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};
  BackUpAction::Feedback::SharedPtr feedback_;
  double target_distance_;
  double reverse_speed_;
};

}  // namespace inflation_backout

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(inflation_backout::InflationBackout, nav2_core::Behavior)
