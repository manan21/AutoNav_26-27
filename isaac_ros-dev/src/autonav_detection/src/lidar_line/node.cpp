#include "autonav_interfaces/msg/line_points.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

template <typename T>
T readValue(const uint8_t * ptr)
{
  T value;
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

std::optional<double> readNumericField(
  const uint8_t * point,
  const sensor_msgs::msg::PointField & field)
{
  const uint8_t * ptr = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(readValue<int8_t>(ptr));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(readValue<uint8_t>(ptr));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(readValue<int16_t>(ptr));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(readValue<uint16_t>(ptr));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(readValue<int32_t>(ptr));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(readValue<uint32_t>(ptr));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(readValue<float>(ptr));
    case sensor_msgs::msg::PointField::FLOAT64:
      return readValue<double>(ptr);
    default:
      return std::nullopt;
  }
}

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

std::uint64_t gridKey(int x, int y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
         static_cast<std::uint32_t>(y);
}

Eigen::Affine3f transformToEigen(const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & tr = tf.transform.translation;
  const auto & rot = tf.transform.rotation;
  Eigen::Quaternionf q(
    static_cast<float>(rot.w),
    static_cast<float>(rot.x),
    static_cast<float>(rot.y),
    static_cast<float>(rot.z));
  if (q.norm() > 1e-6f) {
    q.normalize();
  } else {
    q = Eigen::Quaternionf::Identity();
  }

  Eigen::Affine3f out = Eigen::Affine3f::Identity();
  out.translation() = Eigen::Vector3f(
    static_cast<float>(tr.x),
    static_cast<float>(tr.y),
    static_cast<float>(tr.z));
  out.linear() = q.toRotationMatrix();
  return out;
}

struct RunningStats
{
  int count = 0;
  double sum = 0.0;
  double sum_sq = 0.0;

  void add(double value)
  {
    ++count;
    sum += value;
    sum_sq += value * value;
  }

  double mean() const
  {
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
  }

  double stddev() const
  {
    if (count < 2) {
      return 0.0;
    }
    const double m = mean();
    const double variance =
      std::max(0.0, sum_sq / static_cast<double>(count) - m * m);
    return std::sqrt(variance);
  }
};

}  // namespace

namespace autonav_detection {

class LidarLineDetectorNode : public rclcpp::Node
{
public:
  LidarLineDetectorNode()
  : rclcpp::Node("lidar_line_detector"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    last_process_time_(0, 0, this->get_clock()->get_clock_type())
  {
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/cloud_all_fields_fullframe");
    line_points_topic_ = declare_parameter<std::string>(
      "line_points_topic", "/lidar_line_points");
    debug_points_topic_ = declare_parameter<std::string>(
      "debug_points_topic", "/lidar_line_detection/debug/points");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/lidar_line_detection/diagnostics");
    intensity_field_ = declare_parameter<std::string>("intensity_field", "i");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");

    tf_lookup_timeout_ms_ = declare_parameter<int>("tf_lookup_timeout_ms", 100);
    tf_use_latest_ = declare_parameter<bool>("tf_use_latest", false);
    max_processing_rate_hz_ = declare_parameter<double>(
      "max_processing_rate_hz", 10.0);
    publish_empty_messages_ = declare_parameter<bool>(
      "publish_empty_messages", true);
    publish_debug_points_ = declare_parameter<bool>(
      "publish_debug_points", true);

    range_min_m_ = declare_parameter<double>("range_min_m", 0.20);
    range_max_m_ = declare_parameter<double>("range_max_m", 6.0);
    base_min_x_m_ = declare_parameter<double>("base_min_x_m", -0.10);
    base_max_x_m_ = declare_parameter<double>("base_max_x_m", 5.0);
    base_max_abs_y_m_ = declare_parameter<double>("base_max_abs_y_m", 3.0);
    ground_z_m_ = declare_parameter<double>("ground_z_m", -0.11);
    ground_z_tolerance_m_ = declare_parameter<double>(
      "ground_z_tolerance_m", 0.18);
    layer_min_ = declare_parameter<int>("layer_min", -1);
    layer_max_ = declare_parameter<int>("layer_max", -1);
    echo_filter_ = declare_parameter<int>("echo_filter", -1);

    adaptive_range_bin_m_ = declare_parameter<double>(
      "adaptive_range_bin_m", 0.50);
    adaptive_stddev_multiplier_ = declare_parameter<double>(
      "adaptive_stddev_multiplier", 1.5);
    adaptive_min_delta_ = declare_parameter<double>(
      "adaptive_min_delta", 5.0);
    adaptive_min_samples_ = declare_parameter<int>(
      "adaptive_min_samples", 20);
    min_intensity_ = declare_parameter<double>("min_intensity", 0.0);
    normalize_by_layer_ = declare_parameter<bool>("normalize_by_layer", true);
    use_reflector_boost_ = declare_parameter<bool>(
      "use_reflector_boost", true);
    reflector_threshold_boost_ = declare_parameter<double>(
      "reflector_threshold_boost", 25.0);

    cluster_link_distance_m_ = declare_parameter<double>(
      "cluster_link_distance_m", 0.20);
    cluster_min_points_ = declare_parameter<int>("cluster_min_points", 4);
    cluster_min_length_m_ = declare_parameter<double>(
      "cluster_min_length_m", 0.35);
    cluster_max_width_m_ = declare_parameter<double>(
      "cluster_max_width_m", 0.22);
    cluster_min_aspect_ratio_ = declare_parameter<double>(
      "cluster_min_aspect_ratio", 2.5);
    output_voxel_size_m_ = declare_parameter<double>(
      "output_voxel_size_m", 0.08);
    max_line_points_ = declare_parameter<int>("max_line_points", 8000);

    range_min_m_ = std::max(0.0, range_min_m_);
    range_max_m_ = std::max(range_min_m_, range_max_m_);
    base_max_abs_y_m_ = std::max(0.0, base_max_abs_y_m_);
    ground_z_tolerance_m_ = std::max(0.0, ground_z_tolerance_m_);
    adaptive_range_bin_m_ = std::max(0.05, adaptive_range_bin_m_);
    adaptive_min_samples_ = std::max(1, adaptive_min_samples_);
    cluster_link_distance_m_ = std::max(0.02, cluster_link_distance_m_);
    cluster_min_points_ = std::max(1, cluster_min_points_);
    cluster_min_length_m_ = std::max(0.0, cluster_min_length_m_);
    cluster_max_width_m_ = std::max(0.01, cluster_max_width_m_);
    cluster_min_aspect_ratio_ = std::max(1.0, cluster_min_aspect_ratio_);
    output_voxel_size_m_ = std::max(0.01, output_voxel_size_m_);
    max_line_points_ = std::max(1, max_line_points_);

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos,
      std::bind(&LidarLineDetectorNode::cloudCallback, this, std::placeholders::_1));
    line_pub_ = create_publisher<autonav_interfaces::msg::LinePoints>(
      line_points_topic_, 1);
    debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      debug_points_topic_, qos);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      diagnostics_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "lidar_line_detector up. cloud=%s out=%s target=%s intensity_field=%s",
      cloud_topic_.c_str(), line_points_topic_.c_str(),
      target_frame_.c_str(), intensity_field_.c_str());
  }

private:
  struct Sample
  {
    Eigen::Vector3f lidar;
    Eigen::Vector3f base;
    Eigen::Vector3f target;
    double intensity = 0.0;
    double range = 0.0;
    int layer = -1;
    int echo = -1;
    bool reflector = false;
    double score = 0.0;
  };

  bool lookupTransform(
    const std::string & target,
    const std::string & source,
    const builtin_interfaces::msg::Time & stamp,
    Eigen::Affine3f & out)
  {
    try {
      rclcpp::Time lookup_time(stamp, get_clock()->get_clock_type());
      if (tf_use_latest_ || lookup_time.nanoseconds() == 0) {
        lookup_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      }
      const auto tf = tf_buffer_->lookupTransform(
        target, source, lookup_time,
        rclcpp::Duration::from_nanoseconds(
          static_cast<int64_t>(tf_lookup_timeout_ms_) * 1000000LL));
      out = transformToEigen(tf);
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "TF %s <- %s unavailable: %s",
        target.c_str(), source.c_str(), ex.what());
      return false;
    }
  }

  int statsKey(const Sample & sample) const
  {
    const int range_bin = static_cast<int>(
      std::floor(sample.range / adaptive_range_bin_m_));
    const int layer_bin =
      normalize_by_layer_ && sample.layer >= 0 ? sample.layer : 0;
    return layer_bin * 10000 + range_bin;
  }

  bool passLayerEcho(const Sample & sample) const
  {
    if (layer_min_ >= 0 && sample.layer >= 0 && sample.layer < layer_min_) {
      return false;
    }
    if (layer_max_ >= 0 && sample.layer >= 0 && sample.layer > layer_max_) {
      return false;
    }
    if (echo_filter_ >= 0 && sample.echo >= 0 && sample.echo != echo_filter_) {
      return false;
    }
    return true;
  }

  std::vector<Sample> extractGroundSamples(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const Eigen::Affine3f & lidar_to_base,
    const Eigen::Affine3f & lidar_to_target)
  {
    const auto * field_x = findField(cloud, "x");
    const auto * field_y = findField(cloud, "y");
    const auto * field_z = findField(cloud, "z");
    const auto * field_i = findField(cloud, intensity_field_);
    if (!field_i && intensity_field_ != "intensity") {
      field_i = findField(cloud, "intensity");
    }
    const auto * field_range = findField(cloud, "range");
    const auto * field_layer = findField(cloud, "layer");
    if (!field_layer) {
      field_layer = findField(cloud, "ring");
    }
    const auto * field_echo = findField(cloud, "echo");
    const auto * field_reflector = findField(cloud, "reflector");

    std::vector<Sample> samples;
    if (!field_x || !field_y || !field_z || !field_i) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "PointCloud2 missing required fields x/y/z/%s.",
        intensity_field_.c_str());
      return samples;
    }

    const std::size_t point_count =
      static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
    samples.reserve(point_count / 8);

    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t col = 0; col < cloud.width; ++col) {
        const std::size_t offset =
          static_cast<std::size_t>(row) * cloud.row_step +
          static_cast<std::size_t>(col) * cloud.point_step;
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }
        const uint8_t * point = cloud.data.data() + offset;

        const auto x = readNumericField(point, *field_x);
        const auto y = readNumericField(point, *field_y);
        const auto z = readNumericField(point, *field_z);
        const auto intensity = readNumericField(point, *field_i);
        if (!x || !y || !z || !intensity) {
          continue;
        }

        Sample sample;
        sample.lidar = Eigen::Vector3f(
          static_cast<float>(*x),
          static_cast<float>(*y),
          static_cast<float>(*z));
        sample.base = lidar_to_base * sample.lidar;
        sample.target = lidar_to_target * sample.lidar;
        sample.intensity = *intensity;
        if (field_range) {
          const auto r = readNumericField(point, *field_range);
          sample.range = r ? *r : sample.lidar.norm();
        } else {
          sample.range = sample.lidar.norm();
        }
        if (field_layer) {
          const auto layer = readNumericField(point, *field_layer);
          sample.layer = layer ? static_cast<int>(std::llround(*layer)) : -1;
        }
        if (field_echo) {
          const auto echo = readNumericField(point, *field_echo);
          sample.echo = echo ? static_cast<int>(std::llround(*echo)) : -1;
        }
        if (field_reflector) {
          const auto reflector = readNumericField(point, *field_reflector);
          sample.reflector = reflector && *reflector > 0.5;
        }

        if (!std::isfinite(sample.lidar.x()) ||
            !std::isfinite(sample.lidar.y()) ||
            !std::isfinite(sample.lidar.z()) ||
            !std::isfinite(sample.intensity) ||
            !std::isfinite(sample.range)) {
          continue;
        }
        if (!passLayerEcho(sample)) {
          continue;
        }
        if (sample.range < range_min_m_ || sample.range > range_max_m_) {
          continue;
        }
        if (sample.base.x() < base_min_x_m_ ||
            sample.base.x() > base_max_x_m_ ||
            std::abs(sample.base.y()) > base_max_abs_y_m_) {
          continue;
        }
        if (std::abs(sample.base.z() - ground_z_m_) > ground_z_tolerance_m_) {
          continue;
        }
        samples.push_back(sample);
      }
    }

    return samples;
  }

  std::vector<Sample> selectBrightCandidates(
    const std::vector<Sample> & samples,
    int & adaptive_bins_used)
  {
    RunningStats global_stats;
    std::unordered_map<int, RunningStats> bin_stats;
    bin_stats.reserve(samples.size() / 8 + 1);

    for (const auto & sample : samples) {
      global_stats.add(sample.intensity);
      bin_stats[statsKey(sample)].add(sample.intensity);
    }

    std::vector<Sample> candidates;
    candidates.reserve(samples.size() / 10 + 1);
    adaptive_bins_used = 0;
    for (const auto & sample : samples) {
      const auto bin_it = bin_stats.find(statsKey(sample));
      const RunningStats * stats = &global_stats;
      if (bin_it != bin_stats.end() &&
          bin_it->second.count >= adaptive_min_samples_) {
        stats = &bin_it->second;
        ++adaptive_bins_used;
      }

      double threshold = stats->mean() + std::max(
        adaptive_min_delta_,
        adaptive_stddev_multiplier_ * stats->stddev());
      threshold = std::max(threshold, min_intensity_);
      if (use_reflector_boost_ && sample.reflector) {
        threshold -= reflector_threshold_boost_;
      }

      if (sample.intensity >= threshold) {
        Sample candidate = sample;
        candidate.score = sample.intensity - threshold;
        candidates.push_back(candidate);
      }
    }

    return candidates;
  }

  std::vector<std::vector<int>> clusterCandidates(
    const std::vector<Sample> & candidates)
  {
    std::unordered_map<std::uint64_t, std::vector<int>> grid;
    grid.reserve(candidates.size() * 2 + 1);
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const int gx = static_cast<int>(
        std::floor(candidates[i].base.x() / cluster_link_distance_m_));
      const int gy = static_cast<int>(
        std::floor(candidates[i].base.y() / cluster_link_distance_m_));
      grid[gridKey(gx, gy)].push_back(static_cast<int>(i));
    }

    std::vector<uint8_t> visited(candidates.size(), 0);
    std::vector<std::vector<int>> clusters;
    const float eps_sq = static_cast<float>(
      cluster_link_distance_m_ * cluster_link_distance_m_);

    for (std::size_t start = 0; start < candidates.size(); ++start) {
      if (visited[start]) {
        continue;
      }
      visited[start] = 1;
      std::vector<int> cluster;
      std::queue<int> q;
      q.push(static_cast<int>(start));

      while (!q.empty()) {
        const int idx = q.front();
        q.pop();
        cluster.push_back(idx);

        const int gx = static_cast<int>(
          std::floor(candidates[idx].base.x() / cluster_link_distance_m_));
        const int gy = static_cast<int>(
          std::floor(candidates[idx].base.y() / cluster_link_distance_m_));
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const auto it = grid.find(gridKey(gx + dx, gy + dy));
            if (it == grid.end()) {
              continue;
            }
            for (const int nb : it->second) {
              if (visited[nb]) {
                continue;
              }
              const Eigen::Vector2f d =
                candidates[nb].base.head<2>() - candidates[idx].base.head<2>();
              if (d.squaredNorm() <= eps_sq) {
                visited[nb] = 1;
                q.push(nb);
              }
            }
          }
        }
      }

      clusters.push_back(std::move(cluster));
    }

    return clusters;
  }

  bool clusterLooksLikeLine(
    const std::vector<Sample> & candidates,
    const std::vector<int> & cluster,
    double & length,
    double & width,
    double & aspect)
  {
    length = 0.0;
    width = 0.0;
    aspect = 0.0;
    if (static_cast<int>(cluster.size()) < cluster_min_points_) {
      return false;
    }

    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const int idx : cluster) {
      centroid += candidates[idx].base.head<2>();
    }
    centroid /= static_cast<float>(cluster.size());

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const int idx : cluster) {
      const Eigen::Vector2f d = candidates[idx].base.head<2>() - centroid;
      cov += d * d.transpose();
    }
    cov /= std::max(1.0f, static_cast<float>(cluster.size() - 1));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
    es.computeDirect(cov);
    Eigen::Vector2f major = es.eigenvectors().col(1);
    Eigen::Vector2f minor = es.eigenvectors().col(0);
    if (major.norm() < 1e-6f || minor.norm() < 1e-6f) {
      return false;
    }
    major.normalize();
    minor.normalize();

    float major_min = std::numeric_limits<float>::max();
    float major_max = std::numeric_limits<float>::lowest();
    float minor_min = std::numeric_limits<float>::max();
    float minor_max = std::numeric_limits<float>::lowest();
    for (const int idx : cluster) {
      const Eigen::Vector2f d = candidates[idx].base.head<2>() - centroid;
      const float u = d.dot(major);
      const float v = d.dot(minor);
      major_min = std::min(major_min, u);
      major_max = std::max(major_max, u);
      minor_min = std::min(minor_min, v);
      minor_max = std::max(minor_max, v);
    }

    length = static_cast<double>(major_max - major_min);
    width = static_cast<double>(minor_max - minor_min);
    aspect = length / std::max(width, 0.01);
    return length >= cluster_min_length_m_ &&
           width <= cluster_max_width_m_ &&
           aspect >= cluster_min_aspect_ratio_;
  }

  std::vector<geometry_msgs::msg::Vector3> acceptedLinePoints(
    const std::vector<Sample> & candidates,
    const std::vector<std::vector<int>> & clusters,
    int & accepted_clusters,
    int & rejected_clusters)
  {
    std::vector<geometry_msgs::msg::Vector3> points;
    std::unordered_set<std::uint64_t> seen;
    accepted_clusters = 0;
    rejected_clusters = 0;

    for (const auto & cluster : clusters) {
      double length = 0.0;
      double width = 0.0;
      double aspect = 0.0;
      if (!clusterLooksLikeLine(candidates, cluster, length, width, aspect)) {
        ++rejected_clusters;
        continue;
      }
      ++accepted_clusters;
      for (const int idx : cluster) {
        const auto & p = candidates[idx].target;
        const int qx = static_cast<int>(std::llround(p.x() / output_voxel_size_m_));
        const int qy = static_cast<int>(std::llround(p.y() / output_voxel_size_m_));
        const auto key = gridKey(qx, qy);
        if (!seen.insert(key).second) {
          continue;
        }
        geometry_msgs::msg::Vector3 out;
        out.x = p.x();
        out.y = p.y();
        out.z = p.z();
        points.push_back(out);
        if (static_cast<int>(points.size()) >= max_line_points_) {
          return points;
        }
      }
    }

    return points;
  }

  void publishDebugCloud(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Vector3> & points)
  {
    if (!publish_debug_points_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = header;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_dense = true;
    out.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
    for (const auto & p : points) {
      *ix = static_cast<float>(p.x);
      *iy = static_cast<float>(p.y);
      *iz = static_cast<float>(p.z);
      ++ix;
      ++iy;
      ++iz;
    }
    debug_pub_->publish(out);
  }

  void publishDiagnostics(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::size_t samples,
    std::size_t candidates,
    std::size_t clusters,
    int accepted_clusters,
    int rejected_clusters,
    std::size_t output_points,
    double elapsed_ms)
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "frame=" << cloud.header.frame_id
       << " raw=" << static_cast<std::size_t>(cloud.width) * cloud.height
       << " ground_samples=" << samples
       << " bright_candidates=" << candidates
       << " clusters=" << clusters
       << " accepted_clusters=" << accepted_clusters
       << " rejected_clusters=" << rejected_clusters
       << " output_points=" << output_points
       << " elapsed_ms=" << elapsed_ms;
    msg.data = ss.str();
    diagnostics_pub_->publish(msg);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const rclcpp::Time start = now();
    if (max_processing_rate_hz_ > 0.0 &&
        last_process_time_.nanoseconds() != 0) {
      const double min_dt = 1.0 / max_processing_rate_hz_;
      if ((start - last_process_time_).seconds() < min_dt) {
        return;
      }
    }
    last_process_time_ = start;

    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Skipping cloud with empty frame_id.");
      return;
    }

    Eigen::Affine3f lidar_to_base;
    Eigen::Affine3f lidar_to_target;
    if (!lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, lidar_to_base)) {
      return;
    }
    if (!lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, lidar_to_target)) {
      return;
    }

    const auto samples = extractGroundSamples(*msg, lidar_to_base, lidar_to_target);
    int adaptive_bins_used = 0;
    const auto candidates = selectBrightCandidates(samples, adaptive_bins_used);
    const auto clusters = clusterCandidates(candidates);
    int accepted_clusters = 0;
    int rejected_clusters = 0;
    auto points = acceptedLinePoints(
      candidates, clusters, accepted_clusters, rejected_clusters);

    std_msgs::msg::Header out_header = msg->header;
    out_header.frame_id = target_frame_;
    autonav_interfaces::msg::LinePoints line_msg;
    line_msg.header = out_header;
    line_msg.points = points;
    if (publish_empty_messages_ || !line_msg.points.empty()) {
      line_pub_->publish(line_msg);
    }
    publishDebugCloud(out_header, points);

    const double elapsed_ms = (now() - start).seconds() * 1000.0;
    publishDiagnostics(
      *msg, samples.size(), candidates.size(), clusters.size(),
      accepted_clusters, rejected_clusters, points.size(), elapsed_ms);

    RCLCPP_DEBUG(
      get_logger(),
      "lidar lines: samples=%zu candidates=%zu bins=%d clusters=%zu accepted=%d points=%zu %.1fms",
      samples.size(), candidates.size(), adaptive_bins_used, clusters.size(),
      accepted_clusters, points.size(), elapsed_ms);
  }

  std::string cloud_topic_;
  std::string line_points_topic_;
  std::string debug_points_topic_;
  std::string diagnostics_topic_;
  std::string intensity_field_;
  std::string base_frame_;
  std::string target_frame_;

  int tf_lookup_timeout_ms_;
  bool tf_use_latest_;
  double max_processing_rate_hz_;
  bool publish_empty_messages_;
  bool publish_debug_points_;

  double range_min_m_;
  double range_max_m_;
  double base_min_x_m_;
  double base_max_x_m_;
  double base_max_abs_y_m_;
  double ground_z_m_;
  double ground_z_tolerance_m_;
  int layer_min_;
  int layer_max_;
  int echo_filter_;

  double adaptive_range_bin_m_;
  double adaptive_stddev_multiplier_;
  double adaptive_min_delta_;
  int adaptive_min_samples_;
  double min_intensity_;
  bool normalize_by_layer_;
  bool use_reflector_boost_;
  double reflector_threshold_boost_;

  double cluster_link_distance_m_;
  int cluster_min_points_;
  double cluster_min_length_m_;
  double cluster_max_width_m_;
  double cluster_min_aspect_ratio_;
  double output_voxel_size_m_;
  int max_line_points_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<autonav_interfaces::msg::LinePoints>::SharedPtr line_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Time last_process_time_;
};

}  // namespace autonav_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonav_detection::LidarLineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
