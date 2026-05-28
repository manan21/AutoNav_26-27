#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

namespace autonav_bt
{

struct Point2D
{
  double x;
  double y;
};

class PathFootprintSafe : public BT::ConditionNode
{
public:
  PathFootprintSafe(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to validate before FollowPath"),
      BT::InputPort<std::string>(
        "costmap_topic", "global_costmap/costmap_raw",
        "Raw global costmap topic used for footprint validation"),
      BT::InputPort<std::string>(
        "global_frame", "map", "Frame of the global costmap"),
      BT::InputPort<std::string>(
        "robot_base_frame", "nav_center",
        "Robot frame used to document the footprint frame"),
      BT::InputPort<std::string>(
        "footprint",
        "[[0.545, 0.41], [0.545, -0.41], [-0.545, -0.41], [-0.545, 0.41]]",
        "Robot footprint vertices in robot_base_frame"),
      BT::InputPort<double>(
        "footprint_padding", 0.05,
        "Padding applied to footprint vertices before collision checks"),
      BT::InputPort<int>(
        "lethal_threshold", 254,
        "Cost value considered lethal"),
      BT::InputPort<int>(
        "inscribed_threshold", 253,
        "Cost value considered inscribed collision"),
      BT::InputPort<bool>(
        "ignore_unknown", true,
        "If true, NO_INFORMATION cells do not reject the path"),
      BT::InputPort<int>(
        "pose_stride", 1,
        "Validate every Nth path pose; 1 validates every pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathFootprintSafe: no path available");
      return BT::NodeStatus::FAILURE;
    }

    std::string costmap_topic = "global_costmap/costmap_raw";
    std::string global_frame = "map";
    std::string robot_base_frame = "nav_center";
    std::string footprint_spec;
    double footprint_padding = 0.05;
    int lethal_threshold = 254;
    int inscribed_threshold = 253;
    bool ignore_unknown = true;
    int pose_stride = 1;

    getInput("costmap_topic", costmap_topic);
    getInput("global_frame", global_frame);
    getInput("robot_base_frame", robot_base_frame);
    getInput("footprint", footprint_spec);
    getInput("footprint_padding", footprint_padding);
    getInput("lethal_threshold", lethal_threshold);
    getInput("inscribed_threshold", inscribed_threshold);
    getInput("ignore_unknown", ignore_unknown);
    getInput("pose_stride", pose_stride);

    (void)robot_base_frame;
    pose_stride = std::max(1, pose_stride);
    const unsigned char collision_threshold = static_cast<unsigned char>(
      std::clamp(std::min(lethal_threshold, inscribed_threshold), 0, 255));

    if (!costmap_sub_ || costmap_topic != costmap_topic_) {
      resetCostmapSubscription(costmap_topic);
    }
    if (costmap_executor_) {
      costmap_executor_->spin_some();
    }

    auto footprint = parseFootprint(footprint_spec);
    if (footprint.size() < 3) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PathFootprintSafe: footprint must contain at least 3 xy points: '%s'",
        footprint_spec.c_str());
      return BT::NodeStatus::FAILURE;
    }
    applyAxisPadding(footprint, footprint_padding);

    nav2_msgs::msg::Costmap::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap = latest_costmap_;
    }
    if (!costmap) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathFootprintSafe: costmap unavailable on '%s'", costmap_topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    for (size_t i = 0; i < path.poses.size(); i += static_cast<size_t>(pose_stride)) {
      geometry_msgs::msg::PoseStamped pose = path.poses[i];
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = path.header.frame_id.empty() ? global_frame : path.header.frame_id;
      }

      if (!transformPose(pose, global_frame)) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "PathFootprintSafe: cannot transform path pose from '%s' to '%s'",
          pose.header.frame_id.c_str(), global_frame.c_str());
        return BT::NodeStatus::FAILURE;
      }

      const auto world_footprint = transformFootprint(footprint, pose);
      if (!footprintIsSafe(
          world_footprint, *costmap, collision_threshold, ignore_unknown))
      {
        const auto & p = pose.pose.position;
        RCLCPP_WARN(
          node_->get_logger(),
          "PathFootprintSafe: rejecting path at pose %zu/%zu (%.2f, %.2f); "
          "footprint overlaps global cost >= %u",
          i, path.poses.size(), p.x, p.y, static_cast<unsigned int>(collision_threshold));
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr costmap_callback_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> costmap_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;
  std::string costmap_topic_;

  void resetCostmapSubscription(const std::string & costmap_topic)
  {
    costmap_topic_ = costmap_topic;
    latest_costmap_.reset();
    costmap_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    costmap_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    costmap_executor_->add_callback_group(
      costmap_callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions options;
    options.callback_group = costmap_callback_group_;
    costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        latest_costmap_ = msg;
      },
      options);
  }

  static std::vector<Point2D> parseFootprint(const std::string & spec)
  {
    static const std::regex number_re(
      R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
    std::vector<double> values;
    for (auto it = std::sregex_iterator(spec.begin(), spec.end(), number_re);
      it != std::sregex_iterator(); ++it)
    {
      values.push_back(std::stod(it->str()));
    }

    std::vector<Point2D> points;
    if (values.size() % 2 != 0) {
      return points;
    }
    points.reserve(values.size() / 2);
    for (size_t i = 0; i + 1 < values.size(); i += 2) {
      points.push_back({values[i], values[i + 1]});
    }
    return points;
  }

  static void applyAxisPadding(std::vector<Point2D> & footprint, double padding)
  {
    if (padding <= 0.0 || footprint.empty()) {
      return;
    }

    double cx = 0.0;
    double cy = 0.0;
    for (const auto & p : footprint) {
      cx += p.x;
      cy += p.y;
    }
    cx /= static_cast<double>(footprint.size());
    cy /= static_cast<double>(footprint.size());

    for (auto & p : footprint) {
      if (std::abs(p.x - cx) > 1e-6) {
        p.x += (p.x > cx) ? padding : -padding;
      }
      if (std::abs(p.y - cy) > 1e-6) {
        p.y += (p.y > cy) ? padding : -padding;
      }
    }
  }

  bool transformPose(geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame)
  {
    if (pose.header.frame_id == target_frame) {
      return true;
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame, pose.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(pose, transformed, tf);
      pose = transformed;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static std::vector<Point2D> transformFootprint(
    const std::vector<Point2D> & footprint,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    std::vector<Point2D> world;
    world.reserve(footprint.size());

    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;

    for (const auto & p : footprint) {
      world.push_back({
        x + p.x * c - p.y * s,
        y + p.x * s + p.y * c});
    }
    return world;
  }

  static bool footprintIsSafe(
    const std::vector<Point2D> & footprint,
    const nav2_msgs::msg::Costmap & costmap,
    unsigned char collision_threshold,
    bool ignore_unknown)
  {
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto & p : footprint) {
      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
    }

    const double resolution = costmap.metadata.resolution;
    const double origin_x = costmap.metadata.origin.position.x;
    const double origin_y = costmap.metadata.origin.position.y;
    const int size_x = static_cast<int>(costmap.metadata.size_x);
    const int size_y = static_cast<int>(costmap.metadata.size_y);

    const int min_mx = static_cast<int>(std::floor((min_x - origin_x) / resolution));
    const int max_mx = static_cast<int>(std::floor((max_x - origin_x) / resolution));
    const int min_my = static_cast<int>(std::floor((min_y - origin_y) / resolution));
    const int max_my = static_cast<int>(std::floor((max_y - origin_y) / resolution));

    if (min_mx < 0 || min_my < 0 || max_mx >= size_x || max_my >= size_y) {
      return false;
    }

    for (int my = min_my; my <= max_my; ++my) {
      const double wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
      for (int mx = min_mx; mx <= max_mx; ++mx) {
        const double wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
        if (!pointInPolygon(wx, wy, footprint)) {
          continue;
        }

        const auto index = static_cast<size_t>(my) * static_cast<size_t>(size_x) +
          static_cast<size_t>(mx);
        if (index >= costmap.data.size()) {
          return false;
        }
        const unsigned char cost = costmap.data[index];
        if (ignore_unknown && cost == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
        if (cost >= collision_threshold) {
          return false;
        }
      }
    }

    return true;
  }

  static bool pointInPolygon(
    double x, double y, const std::vector<Point2D> & polygon)
  {
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const auto & pi = polygon[i];
      const auto & pj = polygon[j];

      if (pointOnSegment(x, y, pi, pj)) {
        return true;
      }

      const bool crosses =
        ((pi.y > y) != (pj.y > y)) &&
        (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x);
      if (crosses) {
        inside = !inside;
      }
    }
    return inside;
  }

  static bool pointOnSegment(
    double x, double y, const Point2D & a, const Point2D & b)
  {
    constexpr double eps = 1e-9;
    const double cross = (x - a.x) * (b.y - a.y) - (y - a.y) * (b.x - a.x);
    if (std::abs(cross) > eps) {
      return false;
    }
    const double dot = (x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y);
    if (dot < -eps) {
      return false;
    }
    const double len_sq = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    return dot <= len_sq + eps;
  }
};

}  // namespace autonav_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autonav_bt::PathFootprintSafe>("PathFootprintSafe");
}
