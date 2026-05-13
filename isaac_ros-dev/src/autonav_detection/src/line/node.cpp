#include "autonav_detection/detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "autonav_interfaces/srv/anv_lines.hpp"
#include "autonav_interfaces/msg/line_points.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <cstring>
#include <queue>
#include <sstream>
#include <unordered_map>

class LineDetectorNode : public rclcpp::Node {

public:

	LineDetectorNode() : Node("lines"),
		tf_buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
		tf_listener(tf_buffer) {

		// Set parameters with ZED camera defaults
		this->declare_parameter("camera_topic", "/zed/zed_node/rgb/color/rect/image");
		this->declare_parameter("depth_camera_topic", "/zed/zed_node/depth/depth_registered");
		this->declare_parameter("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");
		this->declare_parameter("line_points_topic", "line_points");
		this->declare_parameter("target_frame", "odom");
		this->declare_parameter("enable_timer", true);
		this->declare_parameter("publish_interval_ms", 250);
		this->declare_parameter("max_rgb_depth_delta_ms", 120);
		this->declare_parameter("tf_lookup_timeout_ms", 100);
		this->declare_parameter("line_hold_timeout_ms", 800);
		this->declare_parameter("line_memory_max_points", 12000);
		this->declare_parameter("roi_min_y_fraction", 0.35);
		this->declare_parameter("max_depth_m", 6.0);
		this->declare_parameter("base_min_x_m", -0.25);
		this->declare_parameter("base_max_x_m", 5.0);
		this->declare_parameter("base_max_abs_y_m", 3.0);
		this->declare_parameter("ground_z_m", -0.11);
		this->declare_parameter("ground_z_tolerance_m", 0.25);
		this->declare_parameter("cluster_min_points", 15);
		this->declare_parameter("cluster_min_length_m", 0.30);
		this->declare_parameter("cluster_max_width_m", 0.25);
		this->declare_parameter("cluster_min_aspect_ratio", 3.0);
		this->declare_parameter("cluster_link_distance_m", 0.18);
		this->declare_parameter("temporal_voxel_size_m", 0.10);
		this->declare_parameter("temporal_min_hits", 2);
		this->declare_parameter("temporal_confirm_window_ms", 750);
		this->declare_parameter("confirmed_hold_ms", 3500);
		this->declare_parameter("odom_topic", "/local_ekf/odom");
		this->declare_parameter("yaw_rate_gate_rad_s", 0.6);
		this->declare_parameter("debug_image_publish_enabled", true);
		this->declare_parameter("debug_image_write_enabled", false);

		// CERIAS line-pixel detector knobs (previously hardcoded as #defines
		// in cuda.cu; now plumbed through line_detector.yaml).
		this->declare_parameter("brightness_threshold", 220.0);  // 0-255 grayscale
		this->declare_parameter("half_window_size", 3);          // window = 2N+1
		this->declare_parameter("sigma_threshold", 5.0);         // local stddev cap
		this->declare_parameter("mew_threshold", 200.0);         // local mean floor

		std::string camera_topic = this->get_parameter("camera_topic").as_string();
		std::string depth_camera_topic = this->get_parameter("depth_camera_topic").as_string();
		std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
		std::string line_points_topic = this->get_parameter("line_points_topic").as_string();
		std::string odom_topic = this->get_parameter("odom_topic").as_string();
		target_frame_ = this->get_parameter("target_frame").as_string();
		this->get_parameter("enable_timer", enable_timer_);
		publish_interval_ms_ = std::max<int64_t>(50, this->get_parameter("publish_interval_ms").as_int());
		max_rgb_depth_delta_ms_ = std::max<int64_t>(0, this->get_parameter("max_rgb_depth_delta_ms").as_int());
		tf_lookup_timeout_ms_ = std::max<int64_t>(0, this->get_parameter("tf_lookup_timeout_ms").as_int());
		line_hold_timeout_ms_ = std::max<int64_t>(0, this->get_parameter("line_hold_timeout_ms").as_int());
		line_memory_max_points_ = std::max<int64_t>(1, this->get_parameter("line_memory_max_points").as_int());
		roi_min_y_fraction_ = std::clamp(this->get_parameter("roi_min_y_fraction").as_double(), 0.0, 1.0);
		max_depth_m_ = std::max(0.1, this->get_parameter("max_depth_m").as_double());
		base_min_x_m_ = this->get_parameter("base_min_x_m").as_double();
		base_max_x_m_ = this->get_parameter("base_max_x_m").as_double();
		base_max_abs_y_m_ = std::max(0.0, this->get_parameter("base_max_abs_y_m").as_double());
		ground_z_m_ = this->get_parameter("ground_z_m").as_double();
		ground_z_tolerance_m_ = std::max(0.0, this->get_parameter("ground_z_tolerance_m").as_double());
		cluster_min_points_ = std::max<int>(1, this->get_parameter("cluster_min_points").as_int());
		cluster_min_length_m_ = std::max(0.0, this->get_parameter("cluster_min_length_m").as_double());
		cluster_max_width_m_ = std::max(0.01, this->get_parameter("cluster_max_width_m").as_double());
		cluster_min_aspect_ratio_ = std::max(1.0, this->get_parameter("cluster_min_aspect_ratio").as_double());
		cluster_link_distance_m_ = std::max(0.01, this->get_parameter("cluster_link_distance_m").as_double());
		temporal_voxel_size_m_ = std::max(0.01, this->get_parameter("temporal_voxel_size_m").as_double());
		temporal_min_hits_ = std::max<int>(1, this->get_parameter("temporal_min_hits").as_int());
		temporal_confirm_window_ms_ = std::max<int64_t>(0, this->get_parameter("temporal_confirm_window_ms").as_int());
		confirmed_hold_ms_ = std::max<int64_t>(0, this->get_parameter("confirmed_hold_ms").as_int());
		yaw_rate_gate_rad_s_ = std::max<double>(0.0, this->get_parameter("yaw_rate_gate_rad_s").as_double());
		debug_image_publish_enabled_ = this->get_parameter("debug_image_publish_enabled").as_bool();
		debug_image_write_enabled_ = this->get_parameter("debug_image_write_enabled").as_bool();
		brightness_threshold_ = this->get_parameter("brightness_threshold").as_double();
		half_window_size_ = std::max<int>(1, this->get_parameter("half_window_size").as_int());
		sigma_threshold_ = static_cast<float>(this->get_parameter("sigma_threshold").as_double());
		mew_threshold_ = static_cast<float>(this->get_parameter("mew_threshold").as_double());

		RCLCPP_INFO(this->get_logger(), "Line Detection Config");
		RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_camera_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Camera info: %s", camera_info_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Output topic: %s", line_points_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
		RCLCPP_INFO(this->get_logger(), "Timer enabled: %s", enable_timer_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "Publish interval: %ld ms", publish_interval_ms_);
		RCLCPP_INFO(this->get_logger(), "RGB/depth max delta: %ld ms", max_rgb_depth_delta_ms_);
		RCLCPP_INFO(this->get_logger(), "TF lookup timeout: %ld ms", tf_lookup_timeout_ms_);
		RCLCPP_INFO(this->get_logger(), "Line hold timeout: %ld ms", line_hold_timeout_ms_);
		RCLCPP_INFO(this->get_logger(), "Line memory max points: %ld", line_memory_max_points_);
		RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic.c_str());
		RCLCPP_INFO(this->get_logger(),
			"Geometry gates: roi_y>=%.2f depth<=%.2fm base_x=[%.2f, %.2f] |base_y|<=%.2f ground=%.2f±%.2f",
			roi_min_y_fraction_, max_depth_m_, base_min_x_m_, base_max_x_m_,
			base_max_abs_y_m_, ground_z_m_, ground_z_tolerance_m_);
		RCLCPP_INFO(this->get_logger(),
			"Cluster gates: min_points=%d min_length=%.2fm max_width=%.2fm min_aspect=%.2f link=%.2fm",
			cluster_min_points_, cluster_min_length_m_, cluster_max_width_m_,
			cluster_min_aspect_ratio_, cluster_link_distance_m_);
		RCLCPP_INFO(this->get_logger(),
			"Temporal gates: voxel=%.2fm min_hits=%d confirm_window=%ld ms hold=%ld ms yaw_gate=%.2f rad/s",
			temporal_voxel_size_m_, temporal_min_hits_, temporal_confirm_window_ms_,
			confirmed_hold_ms_, yaw_rate_gate_rad_s_);
		RCLCPP_INFO(this->get_logger(), "Debug image topics: %s", debug_image_publish_enabled_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "Debug image writes: %s", debug_image_write_enabled_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(),
			"CERIAS knobs: brightness=%.1f half_window=%d sigma<%.2f mew>%.1f",
			brightness_threshold_, half_window_size_, sigma_threshold_, mew_threshold_);
		RCLCPP_INFO(this->get_logger(), "==================================");

		// Subscribe to camera topics
		auto get_latest_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(callback_lock);
			latest_img = msg;
		};
		auto get_latest_depth_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(depth_callback_lock);
			latest_depth_img = msg;
		};
		
		_zed_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
			camera_topic, 10, get_latest_msg);

		_zed_depth_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
			depth_camera_topic, 10, get_latest_depth_msg);

		_camera_model_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			camera_info_topic, 1, std::bind(&LineDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

		_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
			odom_topic, 10, std::bind(&LineDetectorNode::odomCallback, this, std::placeholders::_1));

		// Publishers
		_line_pub = this->create_publisher<autonav_interfaces::msg::LinePoints>(
			line_points_topic, 1);
			
		_line_timer = this->create_wall_timer(
			std::chrono::milliseconds(publish_interval_ms_),
			std::bind(&LineDetectorNode::line_callback, this));

		_line_point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
			"lines_pointcloud", 10);

		// Publish as a stock std_msgs/Int32MultiArray so the native
		// (host-side) HUD can subscribe with only /opt/ros/humble on
		// its Python path — no custom interface dep needed.
		// Wire format: data[0] = image_width, data[1] = image_height,
		// data[2..] is an interleaved [x0, y0, x1, y1, ...] pixel
		// list. The same width/height also appear as layout.dim for
		// callers that prefer the typed accessor.
		_line_pixels_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>(
			"/line_detection/line_pixels", 10);

		_diagnostics_pub = this->create_publisher<std_msgs::msg::String>(
			"/line_detection/diagnostics", 10);

		_debug_raw_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
			"/line_detection/debug/raw", 10);
		_debug_mask_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
			"/line_detection/debug/mask", 10);
		_debug_overlay_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
			"/line_detection/debug/overlay", 10);
			
		// Create service for line detection
		_line_service = this->create_service<autonav_interfaces::srv::AnvLines>(
			"line_service",
			std::bind(&LineDetectorNode::line_service, this, std::placeholders::_1, std::placeholders::_2));

		_clear_lines_service = this->create_service<std_srvs::srv::Trigger>(
			"clear_remembered_lines",
			std::bind(&LineDetectorNode::clearRememberedLines, this, std::placeholders::_1, std::placeholders::_2));
			
		RCLCPP_INFO(this->get_logger(), "LineDetectorNode initialized - waiting for camera data...");
	}

private:

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_depth_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_model_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

	rclcpp::Service<autonav_interfaces::srv::AnvLines>::SharedPtr _line_service;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _clear_lines_service;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _line_point_cloud_pub; 
	rclcpp::Publisher<autonav_interfaces::msg::LinePoints>::SharedPtr _line_pub;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _line_pixels_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _diagnostics_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _debug_raw_image_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _debug_mask_image_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _debug_overlay_image_pub;
	rclcpp::TimerBase::SharedPtr _line_timer;

	std::mutex callback_lock;
	std::mutex depth_callback_lock;
	std::mutex camera_params_lock;
	std::mutex odom_lock;
	
	sensor_msgs::msg::Image::SharedPtr latest_img;
	sensor_msgs::msg::Image::SharedPtr latest_depth_img;
	
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	
	// Store camera intrinsics directly - no PinholeCameraModel to avoid dangling pointer issues
	double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
	
	bool enable_timer_;
	bool configured_ = false;
	std::string target_frame_;
	int64_t publish_interval_ms_ = 250;
	int64_t max_rgb_depth_delta_ms_ = 120;
	int64_t tf_lookup_timeout_ms_ = 100;
	int64_t line_hold_timeout_ms_ = 0;
	int64_t line_memory_max_points_ = 20000;
	double  roi_min_y_fraction_ = 0.35;
	double  max_depth_m_ = 6.0;
	double  base_min_x_m_ = -0.25;
	double  base_max_x_m_ = 5.0;
	double  base_max_abs_y_m_ = 3.0;
	double  ground_z_m_ = -0.11;
	double  ground_z_tolerance_m_ = 0.25;
	int     cluster_min_points_ = 15;
	double  cluster_min_length_m_ = 0.30;
	double  cluster_max_width_m_ = 0.25;
	double  cluster_min_aspect_ratio_ = 3.0;
	double  cluster_link_distance_m_ = 0.18;
	double  temporal_voxel_size_m_ = 0.10;
	int     temporal_min_hits_ = 2;
	int64_t temporal_confirm_window_ms_ = 750;
	int64_t confirmed_hold_ms_ = 3500;
	double  yaw_rate_gate_rad_s_ = 0.6;
	bool    debug_image_publish_enabled_ = true;
	bool    debug_image_write_enabled_ = false;
	double  brightness_threshold_ = 220.0;
	int     half_window_size_ = 3;
	float   sigma_threshold_ = 5.0f;
	float   mew_threshold_ = 200.0f;
	double  latest_yaw_rate_rad_s_ = 0.0;
	bool    has_odom_ = false;
	autonav_interfaces::msg::LinePoints last_valid_message_;
	rclcpp::Time last_valid_detection_time_{0, 0, RCL_ROS_TIME};
	bool has_last_valid_message_ = false;

	struct DetectionFrameStats {
		int raw_pixels = 0;
		int filtered_pixels = 0;
		int kept_components = 0;
		int roi_rejects = 0;
		int depth_rejects = 0;
		int geometry_rejects = 0;
		int out_of_bounds = 0;
		int transform_rejects = 0;
		int cluster_rejects = 0;
		int kept_clusters = 0;
		int candidate_points = 0;
		int confirmed_points = 0;
		int yaw_gated_frames = 0;
	};

	struct CandidatePoint {
		Eigen::Vector3d target;
		Eigen::Vector3d base;
		cv::Point pixel;
	};

	struct VoxelKey {
		int x = 0;
		int y = 0;

		bool operator==(const VoxelKey & other) const {
			return x == other.x && y == other.y;
		}
	};

	struct VoxelKeyHash {
		std::size_t operator()(const VoxelKey & key) const {
			const auto ux = static_cast<uint64_t>(static_cast<uint32_t>(key.x));
			const auto uy = static_cast<uint64_t>(static_cast<uint32_t>(key.y));
			return std::hash<uint64_t>{}((ux << 32) ^ uy);
		}
	};

	struct VoxelState {
		Eigen::Vector3d point;
		cv::Point2d pixel;
		rclcpp::Time first_seen;
		rclcpp::Time last_seen;
		int hits = 0;
		bool confirmed = false;
	};

	std::unordered_map<VoxelKey, VoxelState, VoxelKeyHash> temporal_voxels_;

	void line_service(
		const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
		std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response);
	void clearRememberedLines(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
		std::shared_ptr<std_srvs::srv::Trigger::Response> response);
	
	void line_callback();

	std::vector<CandidatePoint> map_transform(
		const sensor_msgs::msg::Image::SharedPtr depth_msg, 
		int2* line_points, 
		int line_points_len,
		DetectionFrameStats & stats);

	bool imagesAreSynchronized(
		const sensor_msgs::msg::Image::SharedPtr & camera_msg,
		const sensor_msgs::msg::Image::SharedPtr & depth_msg);

	void publishLinePoints(const autonav_interfaces::msg::LinePoints & message);
	void publishPointCloudFromLineMessage(const autonav_interfaces::msg::LinePoints & message);
	void publishEmptyPointCloud(const builtin_interfaces::msg::Time & stamp);
	void publishEmptyLineSet(const builtin_interfaces::msg::Time & stamp, const char * reason);
	std::vector<CandidatePoint> filterLineClusters(
		const std::vector<CandidatePoint> & points,
		DetectionFrameStats & stats) const;
	std::vector<Eigen::Vector3d> updateTemporalConfidence(
		const std::vector<CandidatePoint> & candidates,
		const rclcpp::Time & stamp,
		bool yaw_gated,
		DetectionFrameStats & stats);
	VoxelKey voxelKeyForPoint(const Eigen::Vector3d & point) const;
	bool isYawGated();
	void publishConfirmedOrEmpty(
		const std::vector<Eigen::Vector3d> & points,
		const builtin_interfaces::msg::Time & stamp);
	void publishDiagnostics(const DetectionFrameStats & stats, const char * reason);
	std::vector<cv::Point> confirmedDebugPixels() const;
	void publishDebugImages(
		const sensor_msgs::msg::Image::SharedPtr & camera_msg,
		const cv::Mat & gray_image,
		const int2 * line_points,
		int line_points_len,
		const std::vector<CandidatePoint> & accepted_candidates,
		const std::vector<cv::Point> & confirmed_pixels);

	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

void LineDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(camera_params_lock);
	
	// Extract camera intrinsics
	fx_ = msg->k[0];  // Focal length X
	fy_ = msg->k[4];  // Focal length Y
	cx_ = msg->k[2];  // Principal point X
	cy_ = msg->k[5];  // Principal point Y
	
	if (!configured_) {
		configured_ = true;
		RCLCPP_INFO(this->get_logger(), "Camera parameters initialized: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
			fx_, fy_, cx_, cy_);
		
		if (enable_timer_) {
			RCLCPP_INFO(this->get_logger(), "Publishing enabled on topic: %s", 
				this->get_parameter("line_points_topic").as_string().c_str());
		}
	}
}

void LineDetectorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(odom_lock);
	latest_yaw_rate_rad_s_ = msg->twist.twist.angular.z;
	has_odom_ = true;
}

sensor_msgs::msg::PointCloud2 createPointCloud(
	const std::vector<std::array<float, 3>>& points, 
	const std::string& frame_id,
	const builtin_interfaces::msg::Time& timestamp) 
{
	sensor_msgs::msg::PointCloud2 pointcloud;
	pointcloud.header.frame_id = frame_id;
	pointcloud.header.stamp = timestamp;
	pointcloud.height = 1;
	pointcloud.width = points.size();
	pointcloud.is_dense = false;
	pointcloud.is_bigendian = false;

	sensor_msgs::PointCloud2Modifier modifier(pointcloud);
	modifier.setPointCloud2FieldsByString(1, "xyz");
	modifier.resize(points.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

	for (const auto& point : points) {
		*iter_x = point[0];
		*iter_y = point[1];
		*iter_z = point[2];
		++iter_x;
		++iter_y;
		++iter_z;
	}

	return pointcloud;
}

bool LineDetectorNode::imagesAreSynchronized(
	const sensor_msgs::msg::Image::SharedPtr & camera_msg,
	const sensor_msgs::msg::Image::SharedPtr & depth_msg)
{
	if (!camera_msg || !depth_msg) {
		return false;
	}

	const rclcpp::Time camera_stamp(camera_msg->header.stamp);
	const rclcpp::Time depth_stamp(depth_msg->header.stamp);
	const rclcpp::Duration stamp_delta =
		(camera_stamp >= depth_stamp) ? (camera_stamp - depth_stamp) : (depth_stamp - camera_stamp);
	const rclcpp::Duration max_delta = rclcpp::Duration::from_nanoseconds(
		max_rgb_depth_delta_ms_ * 1000000LL);

	if (stamp_delta > max_delta) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 3000,
			"Skipping line update: RGB/depth timestamps differ by %.1f ms (limit %.1f ms)",
			stamp_delta.seconds() * 1000.0, max_delta.seconds() * 1000.0);
		return false;
	}

	return true;
}

void LineDetectorNode::publishLinePoints(const autonav_interfaces::msg::LinePoints & message)
{
	_line_pub->publish(message);
}

void LineDetectorNode::publishPointCloudFromLineMessage(
	const autonav_interfaces::msg::LinePoints & message)
{
	std::vector<std::array<float, 3>> point_cloud_points;
	point_cloud_points.reserve(message.points.size());
	for (const auto & point : message.points) {
		point_cloud_points.push_back(
			{static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)});
	}

	sensor_msgs::msg::PointCloud2 pc =
		createPointCloud(point_cloud_points, message.header.frame_id, message.header.stamp);
	_line_point_cloud_pub->publish(pc);
}

void LineDetectorNode::publishEmptyPointCloud(const builtin_interfaces::msg::Time & stamp)
{
	sensor_msgs::msg::PointCloud2 pc = createPointCloud({}, target_frame_, stamp);
	_line_point_cloud_pub->publish(pc);
}

void LineDetectorNode::publishEmptyLineSet(
	const builtin_interfaces::msg::Time & stamp,
	const char * reason)
{
	auto empty_message = autonav_interfaces::msg::LinePoints();
	empty_message.header.frame_id = target_frame_;
	// Same rationale as publishConfirmedOrEmpty — stamp with now() so
	// the empty publish doesn't appear "stale" to downstream gates.
	empty_message.header.stamp = this->now();
	last_valid_message_ = empty_message;
	has_last_valid_message_ = false;
	publishEmptyPointCloud(stamp);
	publishLinePoints(empty_message);
	RCLCPP_WARN_THROTTLE(
		get_logger(), *get_clock(), 3000,
		"Publishing empty line set after %s", reason);
}

void LineDetectorNode::clearRememberedLines(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	(void)request;

	has_last_valid_message_ = false;
	last_valid_message_ = autonav_interfaces::msg::LinePoints();
	temporal_voxels_.clear();
	last_valid_detection_time_ = this->now();

	auto empty_message = autonav_interfaces::msg::LinePoints();
	empty_message.header.frame_id = target_frame_;
	empty_message.header.stamp = this->now();
	publishLinePoints(empty_message);
	publishEmptyPointCloud(empty_message.header.stamp);

	response->success = true;
	response->message = "Cleared confirmed line obstacle set";
	RCLCPP_INFO(this->get_logger(), "Cleared confirmed line obstacle set");
}

/**
 * Converts a list of image indices to target frame coordinates.
 */
std::vector<LineDetectorNode::CandidatePoint> LineDetectorNode::map_transform(
	const sensor_msgs::msg::Image::SharedPtr depth_msg, 
	int2* line_points, 
	int line_points_len,
	DetectionFrameStats & stats)
{
	std::vector<CandidatePoint> depth_line_points;
	
	// Early validation
	if (line_points_len <= 0) {
		return depth_line_points;
	}
	
	if (!line_points || !depth_msg || depth_msg->data.empty()) {
		RCLCPP_ERROR(get_logger(), "Invalid input data");
		return depth_line_points;
	}

	// Get camera parameters
	double fx, fy, cx, cy;
	{
		std::lock_guard<std::mutex> lock(camera_params_lock);
		if (!configured_) {
			RCLCPP_ERROR(get_logger(), "Camera not configured!");
			return depth_line_points;
		}
		fx = fx_;
		fy = fy_;
		cx = cx_;
		cy = cy_;
	}

	// Verify encoding
	if (depth_msg->encoding != "32FC1") {
		RCLCPP_ERROR(get_logger(), "Unexpected depth encoding: %s (expected 32FC1)", 
			depth_msg->encoding.c_str());
		return depth_line_points;
	}

	const size_t row_step = depth_msg->step;
	const size_t bytes_per_pixel = sizeof(float);
	const uint8_t* depth_ptr_u8 = depth_msg->data.data();
	std::string frame_id = depth_msg->header.frame_id;

	// Check if transform is available
	bool transform_available = false;
	geometry_msgs::msg::TransformStamped target_transform;
	geometry_msgs::msg::TransformStamped base_transform;
	
	try {
		const rclcpp::Time depth_stamp(depth_msg->header.stamp);
		target_transform = tf_buffer.lookupTransform(
			target_frame_,
			frame_id, 
			depth_stamp,
			rclcpp::Duration::from_nanoseconds(tf_lookup_timeout_ms_ * 1000000LL)
		);
		base_transform = tf_buffer.lookupTransform(
			"base_link",
			frame_id,
			depth_stamp,
			rclcpp::Duration::from_nanoseconds(tf_lookup_timeout_ms_ * 1000000LL)
		);
		transform_available = true;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
			"TF not available at image stamp (%s/base_link <- %s): %s",
			target_frame_.c_str(), frame_id.c_str(), ex.what());
	}

	int valid_count = 0;
	int tf_success = 0;
	const int roi_min_y = static_cast<int>(
		std::round(static_cast<double>(depth_msg->height) * roi_min_y_fraction_));

	// Process each line point
	for (int i = 0; i < line_points_len; i++) {
		if (line_points[i].y < roi_min_y) {
			stats.roi_rejects++;
			continue;
		}

		// Bounds checking
		if (line_points[i].x < 0 || line_points[i].x >= (int)depth_msg->width ||
			line_points[i].y < 0 || line_points[i].y >= (int)depth_msg->height) {
			stats.out_of_bounds++;
			continue;
		}

		// Get depth value (meters)
		const size_t offset = (size_t)line_points[i].y * row_step + (size_t)line_points[i].x * bytes_per_pixel;
		if (offset + sizeof(float) > depth_msg->data.size()) {
			continue;
		}
		
		float depth_m;
		std::memcpy(&depth_m, depth_ptr_u8 + offset, sizeof(float));
		
		if (depth_m < 0.1f || depth_m > static_cast<float>(max_depth_m_) ||
			std::isnan(depth_m) || std::isinf(depth_m)) {
			stats.depth_rejects++;
			continue;
		}
		
		valid_count++;

		// Manual 3D proj
		double x_normalized = (line_points[i].x - cx) / fx;
		double y_normalized = (line_points[i].y - cy) / fy;
		
		double px = x_normalized * depth_m;
		double py = y_normalized * depth_m;
		double pz = depth_m;
		
		if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) {
			continue;
		}

		// TF to target frame if available
		if (transform_available) {
			try {
				geometry_msgs::msg::PointStamped camera_point;
				camera_point.header = depth_msg->header;
				camera_point.point.x = px;
				camera_point.point.y = py;
				camera_point.point.z = pz;
				
				geometry_msgs::msg::PointStamped target_point;
				geometry_msgs::msg::PointStamped base_point;
				tf2::doTransform(camera_point, target_point, target_transform);
				tf2::doTransform(camera_point, base_point, base_transform);
				
				const bool finite =
					std::isfinite(target_point.point.x) &&
					std::isfinite(target_point.point.y) &&
					std::isfinite(base_point.point.x) &&
					std::isfinite(base_point.point.y) &&
					std::isfinite(base_point.point.z);
				if (!finite) {
					stats.transform_rejects++;
					continue;
				}

				const bool in_geometry =
					base_point.point.x >= base_min_x_m_ &&
					base_point.point.x <= base_max_x_m_ &&
					std::abs(base_point.point.y) <= base_max_abs_y_m_ &&
					std::abs(base_point.point.z - ground_z_m_) <= ground_z_tolerance_m_;
				if (!in_geometry) {
					stats.geometry_rejects++;
					continue;
				}

				CandidatePoint candidate;
				candidate.target = Eigen::Vector3d(
						target_point.point.x,
						target_point.point.y,
						0.0);
				candidate.base = Eigen::Vector3d(
						base_point.point.x,
						base_point.point.y,
						base_point.point.z);
				candidate.pixel = cv::Point(line_points[i].x, line_points[i].y);
				depth_line_points.emplace_back(candidate);
				tf_success++;
			} catch (const std::exception& ex) {
				stats.transform_rejects++;
			}
		}
	}

	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
		"Processing: %d valid depth, %d depth rejects, %d ROI rejects, %d geometry rejects, %d out-of-bounds -> %d candidate points",
		valid_count, stats.depth_rejects, stats.roi_rejects, stats.geometry_rejects,
		stats.out_of_bounds, tf_success);

	return depth_line_points;
}

std::vector<LineDetectorNode::CandidatePoint> LineDetectorNode::filterLineClusters(
	const std::vector<CandidatePoint> & points,
	DetectionFrameStats & stats) const
{
	std::vector<CandidatePoint> kept;
	if (points.empty()) {
		return kept;
	}

	const double link_distance_sq = cluster_link_distance_m_ * cluster_link_distance_m_;
	std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash> bins;
	bins.reserve(points.size());
	auto cluster_key_for_base = [this](const Eigen::Vector3d & point) {
		return VoxelKey{
			static_cast<int>(std::floor(point.x() / cluster_link_distance_m_)),
			static_cast<int>(std::floor(point.y() / cluster_link_distance_m_))};
	};

	for (int i = 0; i < static_cast<int>(points.size()); ++i) {
		bins[cluster_key_for_base(points[i].base)].push_back(i);
	}

	std::vector<uint8_t> visited(points.size(), 0);
	for (int seed = 0; seed < static_cast<int>(points.size()); ++seed) {
		if (visited[seed]) {
			continue;
		}

		std::vector<int> cluster_indices;
		std::queue<int> q;
		visited[seed] = 1;
		q.push(seed);

		while (!q.empty()) {
			const int current = q.front();
			q.pop();
			cluster_indices.push_back(current);

			const VoxelKey center = cluster_key_for_base(points[current].base);
			for (int dx = -1; dx <= 1; ++dx) {
				for (int dy = -1; dy <= 1; ++dy) {
					const VoxelKey neighbor_key{center.x + dx, center.y + dy};
					auto bin_it = bins.find(neighbor_key);
					if (bin_it == bins.end()) {
						continue;
					}
					for (const int candidate_idx : bin_it->second) {
						if (visited[candidate_idx]) {
							continue;
						}
						const double dx_m = points[current].base.x() - points[candidate_idx].base.x();
						const double dy_m = points[current].base.y() - points[candidate_idx].base.y();
						if ((dx_m * dx_m + dy_m * dy_m) <= link_distance_sq) {
							visited[candidate_idx] = 1;
							q.push(candidate_idx);
						}
					}
				}
			}
		}

		if (static_cast<int>(cluster_indices.size()) < cluster_min_points_) {
			stats.cluster_rejects += static_cast<int>(cluster_indices.size());
			continue;
		}

		std::vector<cv::Point2f> rect_points;
		rect_points.reserve(cluster_indices.size());
		for (const int idx : cluster_indices) {
			rect_points.emplace_back(
				static_cast<float>(points[idx].base.x()),
				static_cast<float>(points[idx].base.y()));
		}

		const cv::RotatedRect rect = cv::minAreaRect(rect_points);
		const double major_axis = std::max(rect.size.width, rect.size.height);
		const double minor_axis = std::max(0.01, static_cast<double>(std::min(rect.size.width, rect.size.height)));
		const double aspect_ratio = major_axis / minor_axis;
		const bool line_like =
			major_axis >= cluster_min_length_m_ &&
			minor_axis <= cluster_max_width_m_ &&
			aspect_ratio >= cluster_min_aspect_ratio_;

		if (!line_like) {
			stats.cluster_rejects += static_cast<int>(cluster_indices.size());
			continue;
		}

		stats.kept_clusters++;
		for (const int idx : cluster_indices) {
			kept.push_back(points[idx]);
		}
	}

	stats.candidate_points = static_cast<int>(kept.size());
	return kept;
}

LineDetectorNode::VoxelKey LineDetectorNode::voxelKeyForPoint(const Eigen::Vector3d & point) const
{
	const double voxel = std::max(0.01, temporal_voxel_size_m_);
	return VoxelKey{
		static_cast<int>(std::floor(point.x() / voxel)),
		static_cast<int>(std::floor(point.y() / voxel))};
}

bool LineDetectorNode::isYawGated()
{
	std::lock_guard<std::mutex> lock(odom_lock);
	return has_odom_ && std::abs(latest_yaw_rate_rad_s_) > yaw_rate_gate_rad_s_;
}

std::vector<Eigen::Vector3d> LineDetectorNode::updateTemporalConfidence(
	const std::vector<CandidatePoint> & candidates,
	const rclcpp::Time & stamp,
	bool yaw_gated,
	DetectionFrameStats & stats)
{
	struct FrameVoxelAggregate {
		Eigen::Vector3d point = Eigen::Vector3d::Zero();
		cv::Point2d pixel{0.0, 0.0};
		int count = 0;
	};

	std::unordered_map<VoxelKey, FrameVoxelAggregate, VoxelKeyHash> frame_voxels;
	frame_voxels.reserve(candidates.size());
	for (const auto & candidate : candidates) {
		const VoxelKey key = voxelKeyForPoint(candidate.target);
		auto & aggregate = frame_voxels[key];
		aggregate.point += candidate.target;
		aggregate.pixel.x += candidate.pixel.x;
		aggregate.pixel.y += candidate.pixel.y;
		aggregate.count++;
	}

	const rclcpp::Duration confirm_window =
		rclcpp::Duration::from_nanoseconds(temporal_confirm_window_ms_ * 1000000LL);

	for (auto & item : frame_voxels) {
		const VoxelKey & key = item.first;
		const int count = std::max(1, item.second.count);
		Eigen::Vector3d point = item.second.point / count;
		cv::Point2d pixel(
			item.second.pixel.x / count,
			item.second.pixel.y / count);
		auto voxel_it = temporal_voxels_.find(key);

		if (yaw_gated && (voxel_it == temporal_voxels_.end() || !voxel_it->second.confirmed)) {
			continue;
		}

		if (voxel_it == temporal_voxels_.end()) {
			VoxelState state;
			state.point = point;
			state.pixel = pixel;
			state.first_seen = stamp;
			state.last_seen = stamp;
			state.hits = 1;
			state.confirmed = temporal_min_hits_ <= 1;
			temporal_voxels_[key] = state;
			continue;
		}

		VoxelState & state = voxel_it->second;
		const bool within_window = (stamp - state.last_seen) <= confirm_window;
		if (!state.confirmed && !within_window) {
			state.first_seen = stamp;
			state.hits = 1;
		} else if (!state.confirmed) {
			state.hits++;
		}
		state.last_seen = stamp;
		state.point = 0.6 * state.point + 0.4 * point;
		state.pixel.x = 0.6 * state.pixel.x + 0.4 * pixel.x;
		state.pixel.y = 0.6 * state.pixel.y + 0.4 * pixel.y;
		if (state.hits >= temporal_min_hits_) {
			state.confirmed = true;
		}
	}

	const rclcpp::Duration hold =
		rclcpp::Duration::from_nanoseconds(confirmed_hold_ms_ * 1000000LL);
	for (auto it = temporal_voxels_.begin(); it != temporal_voxels_.end();) {
		if ((stamp - it->second.last_seen) > hold) {
			it = temporal_voxels_.erase(it);
		} else {
			++it;
		}
	}

	std::vector<Eigen::Vector3d> confirmed;
	confirmed.reserve(std::min(
		temporal_voxels_.size(),
		static_cast<size_t>(line_memory_max_points_)));
	for (const auto & item : temporal_voxels_) {
		if (!item.second.confirmed) {
			continue;
		}
		confirmed.push_back(item.second.point);
		if (confirmed.size() >= static_cast<size_t>(line_memory_max_points_)) {
			break;
		}
	}

	stats.confirmed_points = static_cast<int>(confirmed.size());
	if (yaw_gated) {
		stats.yaw_gated_frames = 1;
	}
	return confirmed;
}

void LineDetectorNode::publishConfirmedOrEmpty(
	const std::vector<Eigen::Vector3d> & points,
	const builtin_interfaces::msg::Time & stamp)
{
	if (points.empty()) {
		publishEmptyLineSet(stamp, "unconfirmed or expired line evidence");
		return;
	}

	auto message = autonav_interfaces::msg::LinePoints();
	message.header.frame_id = target_frame_;
	// Stamp with now(), not the depth frame's timestamp. The points
	// published here are the *current* temporally-confirmed obstacle
	// set (held for `confirmed_hold_ms`); they should pass downstream
	// freshness gates (e.g. line_layer's max_message_age_ms) on every
	// republish, not be rejected because they trace back to a depth
	// frame that's now seconds old.
	message.header.stamp = this->now();
	message.points.reserve(points.size());
	for (const auto & point : points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		message.points.emplace_back(vec_msg);
	}

	last_valid_message_ = message;
	last_valid_detection_time_ = rclcpp::Time(stamp);
	has_last_valid_message_ = true;
	publishPointCloudFromLineMessage(message);
	publishLinePoints(message);
}

void LineDetectorNode::publishDiagnostics(
	const DetectionFrameStats & stats,
	const char * reason)
{
	std_msgs::msg::String msg;
	std::ostringstream out;
	out << "{"
		<< "\"reason\":\"" << reason << "\","
		<< "\"raw_pixels\":" << stats.raw_pixels << ","
		<< "\"filtered_pixels\":" << stats.filtered_pixels << ","
		<< "\"kept_components\":" << stats.kept_components << ","
		<< "\"roi_rejects\":" << stats.roi_rejects << ","
		<< "\"depth_rejects\":" << stats.depth_rejects << ","
		<< "\"geometry_rejects\":" << stats.geometry_rejects << ","
		<< "\"out_of_bounds\":" << stats.out_of_bounds << ","
		<< "\"transform_rejects\":" << stats.transform_rejects << ","
		<< "\"cluster_rejects\":" << stats.cluster_rejects << ","
		<< "\"kept_clusters\":" << stats.kept_clusters << ","
		<< "\"candidate_points\":" << stats.candidate_points << ","
		<< "\"confirmed_points\":" << stats.confirmed_points << ","
		<< "\"yaw_gated_frames\":" << stats.yaw_gated_frames
		<< "}";
	msg.data = out.str();
	_diagnostics_pub->publish(msg);
}

std::vector<cv::Point> LineDetectorNode::confirmedDebugPixels() const
{
	std::vector<cv::Point> pixels;
	pixels.reserve(temporal_voxels_.size());
	for (const auto & item : temporal_voxels_) {
		if (!item.second.confirmed) {
			continue;
		}
		pixels.emplace_back(
			static_cast<int>(std::round(item.second.pixel.x)),
			static_cast<int>(std::round(item.second.pixel.y)));
	}
	return pixels;
}

void LineDetectorNode::publishDebugImages(
	const sensor_msgs::msg::Image::SharedPtr & camera_msg,
	const cv::Mat & gray_image,
	const int2 * line_points,
	int line_points_len,
	const std::vector<CandidatePoint> & accepted_candidates,
	const std::vector<cv::Point> & confirmed_pixels)
{
	if (!debug_image_publish_enabled_ || !camera_msg || gray_image.empty()) {
		return;
	}

	const bool has_subscribers =
		_debug_raw_image_pub->get_subscription_count() > 0 ||
		_debug_mask_image_pub->get_subscription_count() > 0 ||
		_debug_overlay_image_pub->get_subscription_count() > 0;
	if (!has_subscribers) {
		return;
	}

	cv::Mat raw_bgr;
	try {
		if (camera_msg->encoding == "bgr8") {
			raw_bgr = cv_bridge::toCvShare(camera_msg, sensor_msgs::image_encodings::BGR8)->image.clone();
		} else if (camera_msg->encoding == "bgra8") {
			cv::Mat bgra = cv_bridge::toCvShare(camera_msg, sensor_msgs::image_encodings::BGRA8)->image;
			cv::cvtColor(bgra, raw_bgr, cv::COLOR_BGRA2BGR);
		} else {
			cv::cvtColor(gray_image, raw_bgr, cv::COLOR_GRAY2BGR);
		}
	} catch (const cv_bridge::Exception & ex) {
		cv::cvtColor(gray_image, raw_bgr, cv::COLOR_GRAY2BGR);
	}

	cv::Mat mask;
	cv::threshold(gray_image, mask, brightness_threshold_, 255, cv::THRESH_BINARY);

	cv::Mat overlay = raw_bgr.clone();
	const int n = std::max(0, line_points_len);
	for (int i = 0; i < n; ++i) {
		const int x = line_points[i].x;
		const int y = line_points[i].y;
		if (0 <= x && x < overlay.cols && 0 <= y && y < overlay.rows) {
			cv::circle(overlay, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
		}
	}
	for (const auto & candidate : accepted_candidates) {
		const int x = candidate.pixel.x;
		const int y = candidate.pixel.y;
		if (0 <= x && x < overlay.cols && 0 <= y && y < overlay.rows) {
			cv::circle(overlay, candidate.pixel, 3, cv::Scalar(0, 255, 255), -1);
		}
	}
	for (const auto & pixel : confirmed_pixels) {
		if (0 <= pixel.x && pixel.x < overlay.cols && 0 <= pixel.y && pixel.y < overlay.rows) {
			cv::circle(overlay, pixel, 4, cv::Scalar(0, 255, 0), -1);
		}
	}

	const std_msgs::msg::Header header = camera_msg->header;
	if (_debug_raw_image_pub->get_subscription_count() > 0) {
		_debug_raw_image_pub->publish(
			*cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, raw_bgr).toImageMsg());
	}
	if (_debug_mask_image_pub->get_subscription_count() > 0) {
		_debug_mask_image_pub->publish(
			*cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg());
	}
	if (_debug_overlay_image_pub->get_subscription_count() > 0) {
		_debug_overlay_image_pub->publish(
			*cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg());
	}
}

void LineDetectorNode::line_service(
	const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
	std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response)
{
	(void)request;

	// Get latest images
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
	}();
	
	sensor_msgs::msg::Image::SharedPtr depth_camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
	}();

	if (!camera_msg || !depth_camera_msg) {
		RCLCPP_ERROR(this->get_logger(), "No camera images available");
		return;
	}
	if (!imagesAreSynchronized(camera_msg, depth_camera_msg)) {
		return;
	}

	// Convert camera image to grayscale
	cv_bridge::CvImagePtr cv_ptr;
	try {
		if (camera_msg->encoding == "bgra8" || camera_msg->encoding == "bgr8") {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
			cv::Mat gray;
			cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
			cv_ptr->image = gray;
		} else {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);
		}
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	// Detect lines
	lines::LinePixelDetectionStats pixel_stats;
	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(cv_ptr->image,
				brightness_threshold_, half_window_size_,
				sigma_threshold_, mew_threshold_,
				debug_image_write_enabled_, &pixel_stats);
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;

	DetectionFrameStats stats;
	stats.raw_pixels = pixel_stats.raw_pixels;
	stats.filtered_pixels = pixel_stats.filtered_pixels;
	stats.kept_components = pixel_stats.kept_components;
	std::vector<CandidatePoint> transformed_points =
		filterLineClusters(map_transform(depth_camera_msg, line_points, *line_points_len, stats), stats);

	// Populate response
	for (const auto & point: transformed_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.target.x();
		vec_msg.y = point.target.y();
		vec_msg.z = point.target.z();
		response->points.emplace_back(vec_msg);
	}

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

void LineDetectorNode::line_callback()
{
	DetectionFrameStats stats;

	// Check prerequisites
	if (!configured_){
		return;
	}
	
	if (!enable_timer_) return;

	// Get latest images
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
	}();
	
	sensor_msgs::msg::Image::SharedPtr depth_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
	}();

	if (!camera_msg || !depth_msg) {
		publishEmptyLineSet(this->now(), "missing camera/depth image");
		publishDiagnostics(stats, "missing camera/depth image");
		return;
	}
	if (!imagesAreSynchronized(camera_msg, depth_msg)) {
		publishEmptyLineSet(depth_msg->header.stamp, "RGB/depth desynchronization");
		publishDiagnostics(stats, "RGB/depth desynchronization");
		return;
	}

	// Convert to grayscale
	cv_bridge::CvImagePtr cv_ptr;
	try {
		if (camera_msg->encoding == "bgra8" || camera_msg->encoding == "bgr8") {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
			cv::Mat gray;
			cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
			cv_ptr->image = gray;
		} else {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);
		}
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		publishEmptyLineSet(depth_msg->header.stamp, "cv_bridge exception");
		publishDiagnostics(stats, "cv_bridge exception");
		return;
	}
	
	if (cv_ptr->image.empty() || cv_ptr->image.type() != CV_8UC1) {
		RCLCPP_ERROR(this->get_logger(), "Invalid image after conversion");
		publishEmptyLineSet(depth_msg->header.stamp, "invalid grayscale image");
		publishDiagnostics(stats, "invalid grayscale image");
		return;
	}

	// Detect lines
	std::pair<int2*,int*> line_pair;
	lines::LinePixelDetectionStats pixel_stats;
	try {
		line_pair = lines::detect_line_pixels(cv_ptr->image,
				brightness_threshold_, half_window_size_,
				sigma_threshold_, mew_threshold_,
				debug_image_write_enabled_, &pixel_stats);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Line detection failed: %s", e.what());
		publishEmptyLineSet(depth_msg->header.stamp, "line detection failure");
		publishDiagnostics(stats, "line detection failure");
		return;
	}
	
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;
	stats.raw_pixels = pixel_stats.raw_pixels;
	stats.filtered_pixels = pixel_stats.filtered_pixels;
	stats.kept_components = pixel_stats.kept_components;

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Detected %d line pixels", *line_points_len);

	{
		std_msgs::msg::Int32MultiArray px_msg;
		std_msgs::msg::MultiArrayDimension dim_w;
		dim_w.label = "width";
		dim_w.size = camera_msg->width;
		dim_w.stride = 0;
		std_msgs::msg::MultiArrayDimension dim_h;
		dim_h.label = "height";
		dim_h.size = camera_msg->height;
		dim_h.stride = 0;
		px_msg.layout.dim.push_back(dim_w);
		px_msg.layout.dim.push_back(dim_h);
		const int n = *line_points_len;
		px_msg.data.reserve(2 + 2 * n);
		px_msg.data.push_back(static_cast<int32_t>(camera_msg->width));
		px_msg.data.push_back(static_cast<int32_t>(camera_msg->height));
		for (int i = 0; i < n; ++i) {
			px_msg.data.push_back(line_points[i].x);
			px_msg.data.push_back(line_points[i].y);
		}
		_line_pixels_pub->publish(px_msg);
	}

	if (*line_points_len == 0) {
		const rclcpp::Time stamp(depth_msg->header.stamp);
		const bool yaw_gated = isYawGated();
		std::vector<Eigen::Vector3d> confirmed =
			updateTemporalConfidence({}, stamp, yaw_gated, stats);
		publishConfirmedOrEmpty(confirmed, depth_msg->header.stamp);
		publishDebugImages(
			camera_msg, cv_ptr->image, line_points, *line_points_len,
			{}, confirmedDebugPixels());
		publishDiagnostics(stats, "empty line detection");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Transform to target frame
	std::vector<CandidatePoint> transformed_points;
	try {
		transformed_points = map_transform(depth_msg, line_points, *line_points_len, stats);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
		publishEmptyLineSet(depth_msg->header.stamp, "transform failure");
		publishDiagnostics(stats, "transform failure");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	std::vector<CandidatePoint> clustered_points =
		filterLineClusters(transformed_points, stats);

	const rclcpp::Time stamp(depth_msg->header.stamp);
	const bool yaw_gated = isYawGated();
	std::vector<Eigen::Vector3d> confirmed_points =
		updateTemporalConfidence(clustered_points, stamp, yaw_gated, stats);
	publishConfirmedOrEmpty(confirmed_points, depth_msg->header.stamp);
	publishDebugImages(
		camera_msg, cv_ptr->image, line_points, *line_points_len,
		clustered_points, confirmedDebugPixels());
	publishDiagnostics(stats, yaw_gated ? "yaw gated" : "updated");

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Line stability: %zu transformed, %zu clustered, %zu confirmed",
		transformed_points.size(), clustered_points.size(), confirmed_points.size());

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<LineDetectorNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
