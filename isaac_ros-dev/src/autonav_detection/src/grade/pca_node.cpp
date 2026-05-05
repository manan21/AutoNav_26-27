// ROS2 wrapper for the GradeDetector pipeline.
//
// Subscribes to a SICK multiScan PointCloud2 (default
// /cloud_all_fields_fullframe), iterates only the x/y/z fields, looks up
// the static TF from the lidar mount frame to base_link (so the algorithm
// sees a z-up frame regardless of mount orientation — the SICK on this
// robot is mounted upside down per the URDF), runs the detector, then
// publishes obstacle points back in the original lidar frame for Nav2's
// ObstacleLayer to consume via its own TF buffer.
//
// Per terrain-grade-layer-plan.md "What NOT to do":
//   - Don't apply a TF transform to the OUTPUT cloud — publish in the
//     same frame the input came in.
//   - Strip to xyz only — bandwidth saving at 20 Hz × 11.5k points.

#include "autonav_detection/grade_detector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autonav_detection {

class GradeDetectorNode : public rclcpp::Node {
 public:
  GradeDetectorNode() : rclcpp::Node("grade_detector") {
    // ── I/O ──
    cloud_topic_ = this->declare_parameter<std::string>(
        "cloud_topic", "/cloud_all_fields_fullframe");
    obstacle_topic_ = this->declare_parameter<std::string>(
        "obstacle_topic", "/scan_pca_filtered_points");
    grade_map_topic_ = this->declare_parameter<std::string>(
        "grade_map_topic", "/terrain/grade_map");
    surface_normal_topic_ = this->declare_parameter<std::string>(
        "surface_normal_topic", "/pca/surface_normal");
    publish_grade_map_ =
        this->declare_parameter<bool>("publish_grade_map", true);
    publish_surface_normal_ =
        this->declare_parameter<bool>("publish_surface_normal", true);
    base_frame_ = this->declare_parameter<std::string>(
        "base_frame", "base_link");
    tf_lookup_timeout_ms_ = this->declare_parameter<int>(
        "tf_lookup_timeout_ms", 200);

    // ── Algorithm params (defaults match the simulator-validated values) ──
    GradeDetectorParams p;
    p.internal_resolution = static_cast<float>(
        this->declare_parameter<double>("internal_resolution", p.internal_resolution));
    p.grid_half_size = static_cast<float>(
        this->declare_parameter<double>("grid_half_size", p.grid_half_size));
    p.traversable_max_deg = static_cast<float>(
        this->declare_parameter<double>("traversable_max_deg", p.traversable_max_deg));
    p.pca_noise_margin_deg = static_cast<float>(
        this->declare_parameter<double>("pca_noise_margin_deg", p.pca_noise_margin_deg));
    p.pca_max_valid_deg = static_cast<float>(
        this->declare_parameter<double>("pca_max_valid_deg", p.pca_max_valid_deg));
    p.z_ground_band = static_cast<float>(
        this->declare_parameter<double>("z_ground_band", p.z_ground_band));
    p.wall_min_height = static_cast<float>(
        this->declare_parameter<double>("wall_min_height", p.wall_min_height));
    p.min_pca_points =
        this->declare_parameter<int>("min_pca_points", p.min_pca_points);
    p.pca_planarity_max = static_cast<float>(
        this->declare_parameter<double>("pca_planarity_max", p.pca_planarity_max));
    p.wall_adjacent_dilation = this->declare_parameter<int>(
        "wall_adjacent_dilation", p.wall_adjacent_dilation);
    p.spike_height = static_cast<float>(
        this->declare_parameter<double>("spike_height", p.spike_height));
    p.spike_min_elevated =
        this->declare_parameter<int>("spike_min_elevated", p.spike_min_elevated);
    p.dbscan_eps = static_cast<float>(
        this->declare_parameter<double>("dbscan_eps", p.dbscan_eps));
    p.dbscan_min_samples =
        this->declare_parameter<int>("dbscan_min_samples", p.dbscan_min_samples);
    p.min_cluster_size =
        this->declare_parameter<int>("min_cluster_size", p.min_cluster_size);
    p.front_arc_only =
        this->declare_parameter<bool>("front_arc_only", p.front_arc_only);

    detector_ = std::make_unique<GradeDetector>(p);

    // ── ROS plumbing ──
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();

    obstacle_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, qos);
    if (publish_grade_map_) {
      grade_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          grade_map_topic_, rclcpp::QoS(1).transient_local());
    }
    if (publish_surface_normal_) {
      sn_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
          surface_normal_topic_, 1);
    }

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos,
        std::bind(&GradeDetectorNode::cloudCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "grade_detector up. cloud=%s base=%s out=%s "
                "traversable_max_deg=%.1f",
                cloud_topic_.c_str(), base_frame_.c_str(),
                obstacle_topic_.c_str(), p.traversable_max_deg);
  }

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    using clk = std::chrono::steady_clock;
    const auto t_callback_start = clk::now();

    // ── TF: rotate input cloud's frame into base_link orientation ──
    Eigen::Matrix3f R_to_base = Eigen::Matrix3f::Identity();
    if (!ensureRotationToBase(msg->header.frame_id, R_to_base)) {
      return;  // already logged
    }
    const Eigen::Matrix3f R_back = R_to_base.transpose();

    // ── Iterate xyz ──
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    const size_t n = static_cast<size_t>(msg->width) *
                     static_cast<size_t>(msg->height);
    const bool front_only = detector_->params().front_arc_only;
    std::vector<Eigen::Vector3f> cloud_internal;
    cloud_internal.reserve(front_only ? n / 2 : n);
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const Eigen::Vector3f p_internal = R_to_base * Eigen::Vector3f(x, y, z);
      // Front-arc clamp: drop everything behind the lidar (x<0 in the
      // base-link-aligned internal frame). Halves the point count and
      // keeps DBSCAN tractable. Disable via front_arc_only:false.
      if (front_only && p_internal.x() < 0.0f) continue;
      cloud_internal.push_back(p_internal);
    }

    const auto t_input_done = clk::now();
    const long input_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              t_input_done - t_callback_start).count();

    if (cloud_internal.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Empty cloud after filtering NaNs.");
      return;
    }

    // ── Run detector ──
    GradeDetectorResult result;
    const auto t_compute_start = clk::now();
    detector_->compute(cloud_internal, result, publish_grade_map_);
    const auto t_compute_end = clk::now();
    const long total_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              t_compute_end - t_callback_start).count();
    const long compute_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                t_compute_end - t_compute_start).count();

    // Per-step timing breakdown, logged once per second so we can see
    // which stage dominates the callback budget.
    const auto& tm = result.timing;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[grade_detector %ld us] in=%zu cells=%zu ground=%zu cand=%zu cent=%zu | "
        "input=%ld bin=%ld split=%ld pca=%ld spike=%ld dsprep=%ld dbscan=%ld "
        "ovr=%ld emit=%ld map=%ld out_pts=%zu",
        total_us, tm.n_input, tm.n_populated_cells, tm.n_ground_cells,
        tm.n_candidates, tm.n_centroids,
        input_us, tm.cell_binning_us, tm.ground_split_us, tm.pca_us,
        tm.spike_us, tm.dbscan_prep_us, tm.dbscan_us, tm.override_us,
        tm.emit_us, tm.grade_map_us, result.obstacle_points.size());

    if (compute_us > 60000) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Pipeline %ld us (>60 ms RULES.md #8 budget) on %zu pts",
                           compute_us, cloud_internal.size());
    }

    // ── Publish obstacle cloud (transformed back to the input frame) ──
    publishObstacleCloud(*msg, result, R_back);

    // ── Optional debug publishes ──
    if (publish_grade_map_ && grade_map_pub_ && !result.grade_map.empty()) {
      publishGradeMap(*msg, result, R_back);
    }
    if (publish_surface_normal_ && sn_pub_ && result.surface_normal_valid) {
      geometry_msgs::msg::Vector3Stamped m;
      m.header = msg->header;  // base_frame would be more honest but keep
                                // header parity for RVIZ overlay
      const Eigen::Vector3f sn_in_lidar = R_back * result.surface_normal;
      m.vector.x = sn_in_lidar.x();
      m.vector.y = sn_in_lidar.y();
      m.vector.z = sn_in_lidar.z();
      sn_pub_->publish(m);
    }
  }

  // Looks up TF cloud_frame → base_frame on first successful call and
  // caches the rotation matrix. URDF mounts are static so the cache is
  // safe; we still retry if the lookup fails (TF tree may not yet be up).
  bool ensureRotationToBase(const std::string& cloud_frame,
                            Eigen::Matrix3f& R_out) {
    if (cached_frame_ == cloud_frame && rotation_cached_) {
      R_out = R_cached_;
      return true;
    }
    try {
      auto tf = tf_buffer_->lookupTransform(
          base_frame_, cloud_frame, rclcpp::Time(0),
          rclcpp::Duration::from_nanoseconds(
              static_cast<int64_t>(tf_lookup_timeout_ms_) * 1000000LL));
      const auto& q = tf.transform.rotation;
      Eigen::Quaternionf quat(static_cast<float>(q.w),
                              static_cast<float>(q.x),
                              static_cast<float>(q.y),
                              static_cast<float>(q.z));
      R_cached_ = quat.toRotationMatrix();
      cached_frame_ = cloud_frame;
      rotation_cached_ = true;
      R_out = R_cached_;
      RCLCPP_INFO(this->get_logger(),
                  "Cached static rotation %s → %s.",
                  cloud_frame.c_str(), base_frame_.c_str());
      return true;
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "TF %s → %s unavailable: %s",
                           base_frame_.c_str(), cloud_frame.c_str(), ex.what());
      return false;
    }
  }

  void publishObstacleCloud(const sensor_msgs::msg::PointCloud2& src,
                            const GradeDetectorResult& result,
                            const Eigen::Matrix3f& R_back) {
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = src.header;  // same frame_id (lidar_footprint) + stamp
    out->height = 1;
    out->width = static_cast<uint32_t>(result.obstacle_points.size());
    out->is_dense = true;
    out->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(*out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(result.obstacle_points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(*out, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(*out, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(*out, "z");
    for (const auto& p_internal : result.obstacle_points) {
      const Eigen::Vector3f p = R_back * p_internal;
      *ix = p.x(); *iy = p.y(); *iz = p.z();
      ++ix; ++iy; ++iz;
    }
    obstacle_pub_->publish(std::move(out));
  }

  void publishGradeMap(const sensor_msgs::msg::PointCloud2& src,
                       const GradeDetectorResult& result,
                       const Eigen::Matrix3f& R_back) {
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid->header.stamp = src.header.stamp;
    grid->header.frame_id = src.header.frame_id;  // publish in lidar frame;
                                                   // RVIZ will TF it.
    grid->info.resolution = result.grade_map_resolution;
    grid->info.width = static_cast<uint32_t>(result.grade_map_width);
    grid->info.height = static_cast<uint32_t>(result.grade_map_height);
    // Origin is bottom-left in the algorithm's internal frame; project back
    // by sending the same origin xy in the lidar frame and trusting the TF
    // chain. This is approximate (rotation may shift the origin), but the
    // grade_map is debug-only.
    const Eigen::Vector3f origin_internal(result.grade_map_origin_x,
                                          result.grade_map_origin_y, 0.0f);
    const Eigen::Vector3f origin_lidar = R_back * origin_internal;
    grid->info.origin.position.x = origin_lidar.x();
    grid->info.origin.position.y = origin_lidar.y();
    grid->info.origin.position.z = origin_lidar.z();
    grid->info.origin.orientation.w = 1.0;
    grid->data = result.grade_map;
    grade_map_pub_->publish(std::move(grid));
  }

  // ── Members ──
  std::string cloud_topic_;
  std::string obstacle_topic_;
  std::string grade_map_topic_;
  std::string surface_normal_topic_;
  std::string base_frame_;
  bool publish_grade_map_ = true;
  bool publish_surface_normal_ = true;
  int tf_lookup_timeout_ms_ = 200;

  std::unique_ptr<GradeDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grade_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sn_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string cached_frame_;
  Eigen::Matrix3f R_cached_ = Eigen::Matrix3f::Identity();
  bool rotation_cached_ = false;
};

}  // namespace autonav_detection

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonav_detection::GradeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
