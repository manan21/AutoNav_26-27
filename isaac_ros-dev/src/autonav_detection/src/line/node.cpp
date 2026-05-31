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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <cstring>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

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
		this->declare_parameter("publish_interval_ms", 100);
		this->declare_parameter("max_rgb_depth_delta_ms", 120);
		this->declare_parameter("rgb_depth_sync_buffer_size", 12);
		this->declare_parameter("rgb_depth_require_tf_ready", true);
		this->declare_parameter("tf_lookup_timeout_ms", 100);
		this->declare_parameter("tf_wait_for_stamp_ms", 125);
		this->declare_parameter("line_hold_timeout_ms", 8000);
		this->declare_parameter("motion_cache_hold_ms", 8000);
		this->declare_parameter("line_memory_max_points", 12000);
		this->declare_parameter("roi_min_y_fraction", 0.35);
		this->declare_parameter("max_depth_m", 6.0);
		this->declare_parameter("base_min_x_m", -0.25);
		this->declare_parameter("base_max_x_m", 5.0);
		this->declare_parameter("base_max_abs_y_m", 3.0);
		this->declare_parameter("ground_z_m", -0.11);
		this->declare_parameter("ground_z_tolerance_m", 0.35);
		this->declare_parameter("depth_fill_radius_px", 5);
		this->declare_parameter("depth_fill_min_neighbors", 2);
		this->declare_parameter("depth_fill_max_spread_m", 0.60);
		this->declare_parameter("projection_max_points", 8000);
		this->declare_parameter("cluster_min_points", 15);
		this->declare_parameter("cluster_min_length_m", 0.30);
		this->declare_parameter("cluster_max_width_m", 0.25);
		this->declare_parameter("cluster_min_aspect_ratio", 3.0);
		this->declare_parameter("cluster_link_distance_m", 0.18);
		this->declare_parameter("temporal_voxel_size_m", 0.10);
		this->declare_parameter("temporal_min_hits", 1);
		this->declare_parameter("temporal_confirm_window_ms", 750);
		this->declare_parameter("confirmed_hold_ms", 8000);
		this->declare_parameter("odom_topic", "/local_ekf/odom");
		this->declare_parameter("yaw_rate_gate_rad_s", 0.6);
		this->declare_parameter("debug_image_publish_enabled", true);
		this->declare_parameter("debug_image_write_enabled", false);
		// Emergency fallback only. Latest-TF projection is responsive but
		// smears line points during motion because the image/depth frame
		// was captured at an older robot pose. The default stamped lookup
		// waits briefly for the exact depth timestamp, then bridges with
		// cached confirmed lines instead of publishing misregistered points.
		this->declare_parameter("tf_use_latest", false);

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
		rgb_depth_sync_buffer_size_ = std::max<int64_t>(2, this->get_parameter("rgb_depth_sync_buffer_size").as_int());
		rgb_depth_require_tf_ready_ = this->get_parameter("rgb_depth_require_tf_ready").as_bool();
		tf_lookup_timeout_ms_ = std::max<int64_t>(0, this->get_parameter("tf_lookup_timeout_ms").as_int());
		tf_wait_for_stamp_ms_ = std::max<int64_t>(0, this->get_parameter("tf_wait_for_stamp_ms").as_int());
		line_hold_timeout_ms_ = std::max<int64_t>(0, this->get_parameter("line_hold_timeout_ms").as_int());
		motion_cache_hold_ms_ = std::max<int64_t>(0, this->get_parameter("motion_cache_hold_ms").as_int());
		line_memory_max_points_ = std::max<int64_t>(1, this->get_parameter("line_memory_max_points").as_int());
		roi_min_y_fraction_ = std::clamp(this->get_parameter("roi_min_y_fraction").as_double(), 0.0, 1.0);
		max_depth_m_ = std::max(0.1, this->get_parameter("max_depth_m").as_double());
		base_min_x_m_ = this->get_parameter("base_min_x_m").as_double();
		base_max_x_m_ = this->get_parameter("base_max_x_m").as_double();
		base_max_abs_y_m_ = std::max(0.0, this->get_parameter("base_max_abs_y_m").as_double());
		ground_z_m_ = this->get_parameter("ground_z_m").as_double();
		ground_z_tolerance_m_ = std::max(0.0, this->get_parameter("ground_z_tolerance_m").as_double());
		depth_fill_radius_px_ = std::clamp<int>(
			this->get_parameter("depth_fill_radius_px").as_int(), 0, 15);
		depth_fill_min_neighbors_ = std::max<int>(
			1, this->get_parameter("depth_fill_min_neighbors").as_int());
		depth_fill_max_spread_m_ = std::max<double>(
			0.0, this->get_parameter("depth_fill_max_spread_m").as_double());
		projection_max_points_ = std::max<int>(
			1, this->get_parameter("projection_max_points").as_int());
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
		tf_use_latest_ = this->get_parameter("tf_use_latest").as_bool();
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
		RCLCPP_INFO(this->get_logger(), "RGB/depth sync buffer: %ld frames", rgb_depth_sync_buffer_size_);
		RCLCPP_INFO(this->get_logger(), "RGB/depth require TF ready: %s", rgb_depth_require_tf_ready_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "TF lookup timeout: %ld ms", tf_lookup_timeout_ms_);
		RCLCPP_INFO(this->get_logger(), "Stamped TF wait: %ld ms", tf_wait_for_stamp_ms_);
		RCLCPP_INFO(this->get_logger(), "Line hold timeout: %ld ms", line_hold_timeout_ms_);
		RCLCPP_INFO(this->get_logger(), "Motion cache hold: %ld ms", motion_cache_hold_ms_);
		RCLCPP_INFO(this->get_logger(), "Line memory max points: %ld", line_memory_max_points_);
		RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic.c_str());
		RCLCPP_INFO(this->get_logger(),
			"Geometry gates: roi_y>=%.2f depth<=%.2fm base_x=[%.2f, %.2f] |base_y|<=%.2f ground=%.2f±%.2f",
			roi_min_y_fraction_, max_depth_m_, base_min_x_m_, base_max_x_m_,
			base_max_abs_y_m_, ground_z_m_, ground_z_tolerance_m_);
		RCLCPP_INFO(this->get_logger(),
			"Depth fill: radius=%d px min_neighbors=%d max_spread=%.2f m",
			depth_fill_radius_px_, depth_fill_min_neighbors_, depth_fill_max_spread_m_);
		RCLCPP_INFO(this->get_logger(), "Projection max points: %d", projection_max_points_);
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
			image_buffer_.push_back(msg);
			trimImageBuffer(image_buffer_);
		};
		auto get_latest_depth_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(depth_callback_lock);
			latest_depth_img = msg;
			depth_buffer_.push_back(msg);
			trimImageBuffer(depth_buffer_);
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
	std::deque<sensor_msgs::msg::Image::SharedPtr> image_buffer_;
	std::deque<sensor_msgs::msg::Image::SharedPtr> depth_buffer_;
	
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	std::mutex static_tf_lock;
	std::string cached_camera_frame_;
	bool has_base_camera_transform_ = false;
	Eigen::Affine3d T_base_camera_ = Eigen::Affine3d::Identity();
	
	// Store camera intrinsics directly - no PinholeCameraModel to avoid dangling pointer issues
	double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
	
	bool enable_timer_;
	bool configured_ = false;
	std::string target_frame_;
	int64_t publish_interval_ms_ = 100;
	int64_t max_rgb_depth_delta_ms_ = 120;
	int64_t rgb_depth_sync_buffer_size_ = 12;
	bool    rgb_depth_require_tf_ready_ = true;
	int64_t tf_lookup_timeout_ms_ = 100;
	int64_t tf_wait_for_stamp_ms_ = 125;
	int64_t line_hold_timeout_ms_ = 8000;
	int64_t motion_cache_hold_ms_ = 8000;
	int64_t line_memory_max_points_ = 20000;
	double  roi_min_y_fraction_ = 0.35;
	double  max_depth_m_ = 6.0;
	double  base_min_x_m_ = -0.25;
	double  base_max_x_m_ = 5.0;
	double  base_max_abs_y_m_ = 3.0;
	double  ground_z_m_ = -0.11;
	double  ground_z_tolerance_m_ = 0.35;
	int     depth_fill_radius_px_ = 5;
	int     depth_fill_min_neighbors_ = 2;
	double  depth_fill_max_spread_m_ = 0.60;
	int     projection_max_points_ = 8000;
	int     cluster_min_points_ = 15;
	double  cluster_min_length_m_ = 0.30;
	double  cluster_max_width_m_ = 0.25;
	double  cluster_min_aspect_ratio_ = 3.0;
	double  cluster_link_distance_m_ = 0.18;
	double  temporal_voxel_size_m_ = 0.10;
	int     temporal_min_hits_ = 1;
	int64_t temporal_confirm_window_ms_ = 750;
	int64_t confirmed_hold_ms_ = 8000;
	double  yaw_rate_gate_rad_s_ = 0.6;
	bool    debug_image_publish_enabled_ = true;
	bool    debug_image_write_enabled_ = false;
	bool    tf_use_latest_ = false;
	double  brightness_threshold_ = 220.0;
	int     half_window_size_ = 3;
	float   sigma_threshold_ = 5.0f;
	float   mew_threshold_ = 200.0f;
	double  latest_yaw_rate_rad_s_ = 0.0;
	bool    has_odom_ = false;
	autonav_interfaces::msg::LinePoints last_valid_message_;
	rclcpp::Time last_valid_detection_time_{0, 0, RCL_ROS_TIME};
	bool has_last_valid_message_ = false;
	std::vector<cv::Point> last_valid_debug_pixels_;

	struct DetectionFrameStats {
		int raw_pixels = 0;
		int filtered_pixels = 0;
		int kept_components = 0;
		int roi_rejects = 0;
		int depth_rejects = 0;
		int depth_fill_hits = 0;
		int geometry_rejects = 0;
		int out_of_bounds = 0;
		int transform_rejects = 0;
		int cluster_rejects = 0;
		int kept_clusters = 0;
		int candidate_points = 0;
		int confirmed_points = 0;
		int projected_output_count = 0;
		int yaw_gated_frames = 0;
		int tf_wait_failures = 0;
		int tf_latest_fallbacks = 0;
		double total_callback_ms = 0.0;
		double cuda_detect_ms = 0.0;
		double tf_lookup_ms = 0.0;
		double projection_ms = 0.0;
		double temporal_update_ms = 0.0;
		double debug_publish_ms = 0.0;
	};

	struct CandidatePoint {
		Eigen::Vector3d target;
		Eigen::Vector3d base;
		cv::Point pixel;
	};

	struct SynchronizedImagePair {
		sensor_msgs::msg::Image::SharedPtr camera;
		sensor_msgs::msg::Image::SharedPtr depth;
		bool waiting_for_tf = false;
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
	void trimImageBuffer(std::deque<sensor_msgs::msg::Image::SharedPtr> & buffer) const;
	bool hasStampedBaseTransform(const sensor_msgs::msg::Image::SharedPtr & depth_msg);
	SynchronizedImagePair getSynchronizedImages();

	void publishLinePoints(const autonav_interfaces::msg::LinePoints & message);
	void publishPointCloudFromLineMessage(const autonav_interfaces::msg::LinePoints & message);
	void publishEmptyPointCloud(const builtin_interfaces::msg::Time & stamp);
	void publishEmptyLineSet(const builtin_interfaces::msg::Time & stamp, const char * reason);
	// Republish the last successfully published candidate set with a
	// fresh stamp while it is still within `line_hold_timeout_ms`.
	void republishConfirmedCache(const builtin_interfaces::msg::Time & stamp);
	void republishConfirmedCacheFor(
		const builtin_interfaces::msg::Time & stamp,
		int64_t hold_ms,
		const char * missing_reason,
		const char * expired_reason);
	void publishCandidatePoints(
		const std::vector<CandidatePoint> & candidates,
		const builtin_interfaces::msg::Time & stamp);
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
	std::vector<cv::Point> publishedDebugPixels() const;
	void publishDebugImages(
		const sensor_msgs::msg::Image::SharedPtr & camera_msg,
		const cv::Mat & gray_image,
		const int2 * line_points,
		int line_points_len,
		const std::vector<CandidatePoint> & accepted_candidates,
		const std::vector<cv::Point> & published_pixels);

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

void LineDetectorNode::trimImageBuffer(
	std::deque<sensor_msgs::msg::Image::SharedPtr> & buffer) const
{
	const size_t max_size = static_cast<size_t>(rgb_depth_sync_buffer_size_);
	while (buffer.size() > max_size) {
		buffer.pop_front();
	}
}

bool LineDetectorNode::hasStampedBaseTransform(
	const sensor_msgs::msg::Image::SharedPtr & depth_msg)
{
	if (!depth_msg || !rgb_depth_require_tf_ready_) {
		return true;
	}
	try {
		return tf_buffer.canTransform(
			target_frame_,
			"base_link",
			rclcpp::Time(depth_msg->header.stamp),
			rclcpp::Duration::from_nanoseconds(0));
	} catch (const tf2::TransformException &) {
		return false;
	}
}

LineDetectorNode::SynchronizedImagePair LineDetectorNode::getSynchronizedImages()
{
	std::scoped_lock lock(callback_lock, depth_callback_lock);
	if (image_buffer_.empty() || depth_buffer_.empty()) {
		return {latest_img, latest_depth_img, false};
	}

	const int64_t max_delta_ns = max_rgb_depth_delta_ms_ * 1000000LL;
	auto stamp_ns = [](const sensor_msgs::msg::Image::SharedPtr & msg) -> int64_t {
		if (!msg) {
			return 0;
		}
		return static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
			static_cast<int64_t>(msg->header.stamp.nanosec);
	};

	sensor_msgs::msg::Image::SharedPtr best_image;
	sensor_msgs::msg::Image::SharedPtr best_depth;
	size_t best_image_index = 0;
	size_t best_depth_index = 0;
	sensor_msgs::msg::Image::SharedPtr fallback_image;
	sensor_msgs::msg::Image::SharedPtr fallback_depth;
	size_t fallback_image_index = 0;
	size_t fallback_depth_index = 0;

	for (size_t image_index = image_buffer_.size(); image_index-- > 0;) {
		const auto & image = image_buffer_[image_index];
		const int64_t image_ns = stamp_ns(image);
		int64_t nearest_delta_ns = std::numeric_limits<int64_t>::max();
		size_t nearest_depth_index = 0;

		for (size_t depth_index = 0; depth_index < depth_buffer_.size(); ++depth_index) {
			const int64_t depth_ns = stamp_ns(depth_buffer_[depth_index]);
			const int64_t delta_ns =
				(image_ns >= depth_ns) ? (image_ns - depth_ns) : (depth_ns - image_ns);
			if (delta_ns < nearest_delta_ns) {
				nearest_delta_ns = delta_ns;
				nearest_depth_index = depth_index;
			}
		}

		if (nearest_delta_ns <= max_delta_ns) {
			const auto & depth = depth_buffer_[nearest_depth_index];
			if (!fallback_image || !fallback_depth) {
				fallback_image = image;
				fallback_depth = depth;
				fallback_image_index = image_index;
				fallback_depth_index = nearest_depth_index;
			}
			if (hasStampedBaseTransform(depth)) {
				best_image = image;
				best_depth = depth;
				best_image_index = image_index;
				best_depth_index = nearest_depth_index;
				break;
			}
		}
	}

	if (!best_image || !best_depth) {
		if (!fallback_image || !fallback_depth) {
			return {latest_img, latest_depth_img, false};
		}
		if (rgb_depth_require_tf_ready_) {
			return {nullptr, nullptr, true};
		}
		best_image = fallback_image;
		best_depth = fallback_depth;
		best_image_index = fallback_image_index;
		best_depth_index = fallback_depth_index;
	}

	if (best_image_index > 0) {
		image_buffer_.erase(image_buffer_.begin(), image_buffer_.begin() + best_image_index);
	}
	if (best_depth_index > 0) {
		depth_buffer_.erase(depth_buffer_.begin(), depth_buffer_.begin() + best_depth_index);
	}

	return {best_image, best_depth, false};
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
	last_valid_debug_pixels_.clear();
	publishEmptyPointCloud(stamp);
	publishLinePoints(empty_message);
	RCLCPP_WARN_THROTTLE(
		get_logger(), *get_clock(), 3000,
		"Publishing empty line set after %s", reason);
}

void LineDetectorNode::republishConfirmedCache(
	const builtin_interfaces::msg::Time & stamp)
{
	republishConfirmedCacheFor(
		stamp,
		line_hold_timeout_ms_,
		"missing cached candidate line set",
		"expired cached candidate line set");
}

void LineDetectorNode::republishConfirmedCacheFor(
	const builtin_interfaces::msg::Time & stamp,
	int64_t hold_ms,
	const char * missing_reason,
	const char * expired_reason)
{
	const rclcpp::Time now_stamp = this->now();
	const rclcpp::Duration hold =
		rclcpp::Duration::from_nanoseconds(hold_ms * 1000000LL);
	if (!has_last_valid_message_) {
		publishEmptyLineSet(stamp, missing_reason);
		return;
	}
	if (hold_ms >= 0 &&
		(now_stamp - last_valid_detection_time_) > hold)
	{
		publishEmptyLineSet(stamp, expired_reason);
		return;
	}

	auto message = last_valid_message_;
	message.header.stamp = this->now();
	publishPointCloudFromLineMessage(message);
	publishLinePoints(message);
}

void LineDetectorNode::publishCandidatePoints(
	const std::vector<CandidatePoint> & candidates,
	const builtin_interfaces::msg::Time & stamp)
{
	(void)stamp;
	auto message = autonav_interfaces::msg::LinePoints();
	message.header.frame_id = target_frame_;
	message.header.stamp = this->now();  // Use now() to avoid "stale" timestamps on republished messages
	message.points.reserve(candidates.size());

	last_valid_debug_pixels_.clear();
	last_valid_debug_pixels_.reserve(candidates.size());
	for (const auto & candidate : candidates) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = candidate.target.x();
		vec_msg.y = candidate.target.y();
		vec_msg.z = candidate.target.z();
		message.points.emplace_back(vec_msg);
		last_valid_debug_pixels_.push_back(candidate.pixel);
	}

	last_valid_message_ = message;
	// Cache freshness is wall/node-time based. Depth image stamps can
	// arrive old or jitter relative to ROS now; using them here makes a
	// healthy confirmed cache expire during exactly the TF/depth gaps this
	// cache is meant to bridge.
	last_valid_detection_time_ = this->now();
	has_last_valid_message_ = true;
	publishPointCloudFromLineMessage(message);
	publishLinePoints(message);
}

void LineDetectorNode::clearRememberedLines(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	(void)request;

	has_last_valid_message_ = false;
	last_valid_message_ = autonav_interfaces::msg::LinePoints();
	last_valid_debug_pixels_.clear();
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

	auto read_valid_depth = [&](int x, int y, float & depth_m) -> bool {
		if (x < 0 || x >= static_cast<int>(depth_msg->width) ||
			y < 0 || y >= static_cast<int>(depth_msg->height)) {
			return false;
		}
		const size_t offset = static_cast<size_t>(y) * row_step +
			static_cast<size_t>(x) * bytes_per_pixel;
		if (offset + sizeof(float) > depth_msg->data.size()) {
			return false;
		}
		std::memcpy(&depth_m, depth_ptr_u8 + offset, sizeof(float));
		return depth_m >= 0.1f &&
			depth_m <= static_cast<float>(max_depth_m_) &&
			std::isfinite(depth_m);
	};

	auto read_nearest_depth = [&](int x, int y, float & depth_m) -> bool {
		if (read_valid_depth(x, y, depth_m)) {
			return true;
		}
		const int radius = depth_fill_radius_px_;
		std::vector<float> neighbor_depths;
		neighbor_depths.reserve((2 * radius + 1) * (2 * radius + 1));
		for (int dy = -radius; dy <= radius; ++dy) {
			for (int dx = -radius; dx <= radius; ++dx) {
				if (dx == 0 && dy == 0) {
					continue;
				}
				const int dist_sq = dx * dx + dy * dy;
				if (dist_sq > radius * radius) {
					continue;
				}
				float candidate_depth = 0.0f;
				if (!read_valid_depth(x + dx, y + dy, candidate_depth)) {
					continue;
				}
				neighbor_depths.push_back(candidate_depth);
			}
		}
		if (static_cast<int>(neighbor_depths.size()) < depth_fill_min_neighbors_) {
			return false;
		}
		const auto minmax =
			std::minmax_element(neighbor_depths.begin(), neighbor_depths.end());
		if ((*minmax.second - *minmax.first) > depth_fill_max_spread_m_) {
			return false;
		}
		const auto middle = neighbor_depths.begin() + neighbor_depths.size() / 2;
		std::nth_element(neighbor_depths.begin(), middle, neighbor_depths.end());
		depth_m = *middle;
		stats.depth_fill_hits++;
		return true;
	};

	// Stamped TF is the normal path: every projected point uses the robot
	// pose at the depth-image stamp. Cache the camera->base static leg and
	// wait only for target<-base each frame; latest TF remains an explicit
	// emergency fallback only because it can scatter points.
	bool transform_available = false;
	Eigen::Affine3d T_target = Eigen::Affine3d::Identity();
	Eigen::Affine3d T_base = Eigen::Affine3d::Identity();
	
	const auto tf_start = std::chrono::steady_clock::now();
	try {
		{
			std::lock_guard<std::mutex> lock(static_tf_lock);
			if (!has_base_camera_transform_ || cached_camera_frame_ != frame_id) {
				const auto latest_timeout =
					rclcpp::Duration::from_nanoseconds(tf_lookup_timeout_ms_ * 1000000LL);
				const rclcpp::Time latest(0, 0, RCL_ROS_TIME);
				const auto base_camera_transform = tf_buffer.lookupTransform(
					"base_link", frame_id, latest, latest_timeout);
				T_base_camera_ = tf2::transformToEigen(base_camera_transform);
				cached_camera_frame_ = frame_id;
				has_base_camera_transform_ = true;
			}
			T_base = T_base_camera_;
		}

		const rclcpp::Time depth_stamp(depth_msg->header.stamp);
		const auto stamped_timeout =
			rclcpp::Duration::from_nanoseconds(tf_wait_for_stamp_ms_ * 1000000LL);
		const auto target_base_transform = tf_buffer.lookupTransform(
			target_frame_, "base_link", depth_stamp, stamped_timeout);
		T_target = tf2::transformToEigen(target_base_transform) * T_base;
		transform_available = true;
	} catch (const tf2::TransformException& stamped_ex) {
		stats.tf_wait_failures++;
		if (tf_use_latest_) {
			try {
				// Emergency fallback only. This preserves legacy behavior
				// for field debugging but remains disabled by default.
				stats.tf_latest_fallbacks++;
				const auto latest_timeout =
					rclcpp::Duration::from_nanoseconds(tf_lookup_timeout_ms_ * 1000000LL);
				const rclcpp::Time latest(0, 0, RCL_ROS_TIME);
				const auto base_camera_transform = tf_buffer.lookupTransform(
					"base_link", frame_id, latest, latest_timeout);
				const auto target_base_transform = tf_buffer.lookupTransform(
					target_frame_, "base_link", latest, latest_timeout);
				T_base = tf2::transformToEigen(base_camera_transform);
				T_target = tf2::transformToEigen(target_base_transform) * T_base;
				transform_available = true;
			} catch (const tf2::TransformException& latest_ex) {
				RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
					"TF not available (%s/base_link <- %s, stamped wait=%ld ms, latest fallback failed): stamped=%s latest=%s",
					target_frame_.c_str(), frame_id.c_str(), tf_wait_for_stamp_ms_,
					stamped_ex.what(), latest_ex.what());
			}
		} else {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
				"Stamped TF not available (%s/base_link <- %s, wait=%ld ms): %s",
				target_frame_.c_str(), frame_id.c_str(), tf_wait_for_stamp_ms_,
				stamped_ex.what());
		}
	}
	stats.tf_lookup_ms =
		std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - tf_start).count();

	if (!transform_available) {
		return depth_line_points;
	}

	int valid_count = 0;
	int tf_success = 0;
	const int roi_min_y = static_cast<int>(
		std::round(static_cast<double>(depth_msg->height) * roi_min_y_fraction_));
	std::vector<int2> roi_line_points;
	roi_line_points.reserve(static_cast<size_t>(line_points_len));
	for (int i = 0; i < line_points_len; ++i) {
		if (line_points[i].y < roi_min_y) {
			stats.roi_rejects++;
			continue;
		}
		roi_line_points.push_back(line_points[i]);
	}
	const int roi_line_points_len = static_cast<int>(roi_line_points.size());
	if (roi_line_points_len <= 0) {
		return depth_line_points;
	}

	auto to_row_major_float = [](const Eigen::Affine3d & transform, float * out) {
		const Eigen::Matrix4d matrix = transform.matrix();
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				out[r * 4 + c] = static_cast<float>(matrix(r, c));
			}
		}
	};

	const int projection_count = std::min(roi_line_points_len, projection_max_points_);
	if (projection_count > 0) {
		float target_matrix[16];
		float base_matrix[16];
		to_row_major_float(T_target, target_matrix);
		to_row_major_float(T_base, base_matrix);
		std::vector<LineProjectionResult> projection_results(projection_count);
		LineProjectionStats projection_stats;
		const auto projection_start = std::chrono::steady_clock::now();
		const cudaError_t projection_err = project_line_pixels_cuda(
			roi_line_points.data(),
			roi_line_points_len,
			depth_ptr_u8,
			depth_msg->data.size(),
			static_cast<int>(depth_msg->width),
			static_cast<int>(depth_msg->height),
			row_step,
			static_cast<float>(fx),
			static_cast<float>(fy),
			static_cast<float>(cx),
			static_cast<float>(cy),
			target_matrix,
			base_matrix,
			projection_max_points_,
			roi_min_y,
			static_cast<float>(max_depth_m_),
			static_cast<float>(base_min_x_m_),
			static_cast<float>(base_max_x_m_),
			static_cast<float>(base_max_abs_y_m_),
			static_cast<float>(ground_z_m_),
			static_cast<float>(ground_z_tolerance_m_),
			depth_fill_radius_px_,
			depth_fill_min_neighbors_,
			static_cast<float>(depth_fill_max_spread_m_),
			projection_results.data(),
			static_cast<int>(projection_results.size()),
			&projection_stats);
		stats.projection_ms =
			std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - projection_start).count();

		if (projection_err == cudaSuccess) {
			stats.depth_rejects += projection_stats.depth_rejects;
			stats.depth_fill_hits += projection_stats.depth_fill_hits;
			stats.roi_rejects += projection_stats.roi_rejects;
			stats.geometry_rejects += projection_stats.geometry_rejects;
			stats.out_of_bounds += projection_stats.out_of_bounds;
			stats.transform_rejects += projection_stats.transform_rejects;
			stats.projected_output_count = projection_stats.projected_count;
			depth_line_points.reserve(static_cast<size_t>(projection_stats.projected_count));
			for (int i = 0; i < projection_stats.projected_count; ++i) {
				const auto & result = projection_results[i];
				CandidatePoint candidate;
				candidate.target = Eigen::Vector3d(result.target_x, result.target_y, result.target_z);
				candidate.base = Eigen::Vector3d(result.base_x, result.base_y, result.base_z);
				candidate.pixel = cv::Point(result.pixel_x, result.pixel_y);
				depth_line_points.emplace_back(candidate);
			}
			RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
				"GPU projection: %d valid depth, %d depth fills, %d depth rejects, %d ROI rejects, %d geometry rejects, %d out-of-bounds -> %d candidate points",
				projection_stats.valid_depth, projection_stats.depth_fill_hits,
				projection_stats.depth_rejects, projection_stats.roi_rejects,
				projection_stats.geometry_rejects, projection_stats.out_of_bounds,
				projection_stats.projected_count);
			return depth_line_points;
		}

		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
			"GPU line projection failed (%s); falling back to CPU projection",
			cudaGetErrorString(projection_err));
	}

	// CPU fallback uses the same even sampling as the GPU path. Avoid a
	// simple integer stride here: when line_points_len is only slightly
	// above the cap, floor-striding samples a biased prefix of the line.
	const int cpu_projection_count = std::min(roi_line_points_len, projection_max_points_);

	// Process each line point
	const auto projection_start = std::chrono::steady_clock::now();
	for (int sample_idx = 0; sample_idx < cpu_projection_count; ++sample_idx) {
		const int i = std::min(
			roi_line_points_len - 1,
			static_cast<int>(
				(static_cast<int64_t>(sample_idx) * roi_line_points_len) /
				cpu_projection_count));
		const int2 & point = roi_line_points[i];

		// Bounds checking
		if (point.x < 0 || point.x >= (int)depth_msg->width ||
			point.y < 0 || point.y >= (int)depth_msg->height) {
			stats.out_of_bounds++;
			continue;
		}

		float depth_m;
		if (!read_nearest_depth(point.x, point.y, depth_m)) {
			stats.depth_rejects++;
			continue;
		}

		valid_count++;

		// Manual 3D proj — camera optical frame
		double x_normalized = (point.x - cx) / fx;
		double y_normalized = (point.y - cy) / fy;

		double px = x_normalized * depth_m;
		double py = y_normalized * depth_m;
		double pz = depth_m;

		if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) {
			continue;
		}

		if (transform_available) {
			const Eigen::Vector3d p_cam(px, py, pz);
			const Eigen::Vector3d p_target = T_target * p_cam;
			const Eigen::Vector3d p_base = T_base * p_cam;

			if (!std::isfinite(p_target.x()) || !std::isfinite(p_target.y()) ||
				!std::isfinite(p_base.x()) || !std::isfinite(p_base.y()) ||
				!std::isfinite(p_base.z())) {
				stats.transform_rejects++;
				continue;
			}

			const bool in_geometry =
				p_base.x() >= base_min_x_m_ &&
				p_base.x() <= base_max_x_m_ &&
				std::abs(p_base.y()) <= base_max_abs_y_m_ &&
				std::abs(p_base.z() - ground_z_m_) <= ground_z_tolerance_m_;
			if (!in_geometry) {
				stats.geometry_rejects++;
				continue;
			}

			CandidatePoint candidate;
			candidate.target = Eigen::Vector3d(p_target.x(), p_target.y(), 0.0);
			candidate.base = p_base;
			candidate.pixel = cv::Point(point.x, point.y);
			depth_line_points.emplace_back(candidate);
			tf_success++;
		}
	}
	stats.projection_ms =
		std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - projection_start).count();
	stats.projected_output_count = tf_success;

	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
		"Processing: %d valid depth, %d depth fills, %d depth rejects, %d ROI rejects, %d geometry rejects, %d out-of-bounds -> %d candidate points",
		valid_count, stats.depth_fill_hits, stats.depth_rejects, stats.roi_rejects,
		stats.geometry_rejects, stats.out_of_bounds, tf_success);

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

		// SHAPE-AGNOSTIC cluster acceptance. Lines on the AutoNav course
		// can be straight, T/L junctions, or curves; PCA / minAreaRect
		// shape gates assumed a dominant linear direction and rejected
		// curved lines. The image-space CC filter (detection.cpp) already
		// ensures pixel continuity + thinness; the 3D step only needs to
		// confirm enough connected points to be meaningful, then trust
		// upstream classification. Compute bbox extent for the min-length
		// sanity check only.
		double x_min = std::numeric_limits<double>::infinity();
		double x_max = -std::numeric_limits<double>::infinity();
		double y_min = std::numeric_limits<double>::infinity();
		double y_max = -std::numeric_limits<double>::infinity();
		for (const int idx : cluster_indices) {
			const double x = points[idx].base.x();
			const double y = points[idx].base.y();
			x_min = std::min(x_min, x);
			x_max = std::max(x_max, x);
			y_min = std::min(y_min, y);
			y_max = std::max(y_max, y);
		}
		const double bbox_extent = std::max(x_max - x_min, y_max - y_min);
		if (bbox_extent < cluster_min_length_m_) {
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

	if (yaw_gated) {
		stats.yaw_gated_frames = 1;
	}

	std::unordered_map<VoxelKey, FrameVoxelAggregate, VoxelKeyHash> frame_voxels;
	if (!yaw_gated) {
		frame_voxels.reserve(candidates.size());
		for (const auto & candidate : candidates) {
			const VoxelKey key = voxelKeyForPoint(candidate.target);
			auto & aggregate = frame_voxels[key];
			aggregate.point += candidate.target;
			aggregate.pixel.x += candidate.pixel.x;
			aggregate.pixel.y += candidate.pixel.y;
			aggregate.count++;
		}
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
	(void)stamp;

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
	// See publishCandidatePoints: cache expiration must be measured in
	// node time, not depth-frame stamp time, to avoid transient projection
	// gaps publishing empty /line_points.
	last_valid_detection_time_ = this->now();
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
		<< "\"depth_fill_hits\":" << stats.depth_fill_hits << ","
		<< "\"geometry_rejects\":" << stats.geometry_rejects << ","
		<< "\"out_of_bounds\":" << stats.out_of_bounds << ","
		<< "\"transform_rejects\":" << stats.transform_rejects << ","
		<< "\"cluster_rejects\":" << stats.cluster_rejects << ","
		<< "\"kept_clusters\":" << stats.kept_clusters << ","
		<< "\"candidate_points\":" << stats.candidate_points << ","
		<< "\"confirmed_points\":" << stats.confirmed_points << ","
		<< "\"projected_output_count\":" << stats.projected_output_count << ","
		<< "\"yaw_gated_frames\":" << stats.yaw_gated_frames << ","
		<< "\"tf_wait_failures\":" << stats.tf_wait_failures << ","
		<< "\"tf_latest_fallbacks\":" << stats.tf_latest_fallbacks << ","
		<< "\"total_callback_ms\":" << stats.total_callback_ms << ","
		<< "\"cuda_detect_ms\":" << stats.cuda_detect_ms << ","
		<< "\"tf_lookup_ms\":" << stats.tf_lookup_ms << ","
		<< "\"projection_ms\":" << stats.projection_ms << ","
		<< "\"temporal_update_ms\":" << stats.temporal_update_ms << ","
		<< "\"debug_publish_ms\":" << stats.debug_publish_ms
		<< "}";
	msg.data = out.str();
	_diagnostics_pub->publish(msg);
}

std::vector<cv::Point> LineDetectorNode::publishedDebugPixels() const
{
	return last_valid_debug_pixels_;
}

void LineDetectorNode::publishDebugImages(
	const sensor_msgs::msg::Image::SharedPtr & camera_msg,
	const cv::Mat & gray_image,
	const int2 * line_points,
	int line_points_len,
	const std::vector<CandidatePoint> & accepted_candidates,
	const std::vector<cv::Point> & published_pixels)
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

	const std_msgs::msg::Header header = camera_msg->header;
	const bool need_mask = _debug_mask_image_pub->get_subscription_count() > 0;
	const bool need_overlay = _debug_overlay_image_pub->get_subscription_count() > 0;

	if (_debug_raw_image_pub->get_subscription_count() > 0) {
		_debug_raw_image_pub->publish(
			*cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, raw_bgr).toImageMsg());
	}
	if (need_mask) {
		cv::Mat mask;
		cv::threshold(gray_image, mask, brightness_threshold_, 255, cv::THRESH_BINARY);
		_debug_mask_image_pub->publish(
			*cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg());
	}
	if (need_overlay) {
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
		for (const auto & pixel : published_pixels) {
			if (0 <= pixel.x && pixel.x < overlay.cols && 0 <= pixel.y && pixel.y < overlay.rows) {
				cv::circle(overlay, pixel, 4, cv::Scalar(0, 255, 0), -1);
			}
		}
		_debug_overlay_image_pub->publish(
			*cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg());
	}
}

void LineDetectorNode::line_service(
	const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
	std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response)
{
	(void)request;

	const auto image_pair = getSynchronizedImages();
	const auto camera_msg = image_pair.camera;
	const auto depth_camera_msg = image_pair.depth;

	if (!camera_msg || !depth_camera_msg) {
		if (image_pair.waiting_for_tf) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(), *get_clock(), 3000,
				"No synchronized camera pair has TF ready yet");
		} else {
			RCLCPP_ERROR(this->get_logger(), "No camera images available");
		}
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
		map_transform(depth_camera_msg, line_points, *line_points_len, stats);

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
	const auto callback_start = std::chrono::steady_clock::now();
	auto elapsed_ms = [](const auto & start) {
		return std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - start).count();
	};
	auto publish_timed_diagnostics = [&](const char * reason) {
		stats.total_callback_ms = elapsed_ms(callback_start);
		publishDiagnostics(stats, reason);
	};

	// Check prerequisites
	if (!configured_){
		return;
	}
	
	if (!enable_timer_) return;

	const auto image_pair = getSynchronizedImages();
	const auto camera_msg = image_pair.camera;
	const auto depth_msg = image_pair.depth;

	if (!camera_msg || !depth_msg) {
		// Republish cached confirmed voxels (held for confirmed_hold_ms)
		// instead of blanking the costmap. Matches lidar's cached-cloud
		// republish pattern so the local_costmap sees stable line
		// obstacles across transient input gaps.
		republishConfirmedCache(this->now());
		publish_timed_diagnostics(
			image_pair.waiting_for_tf
				? "waiting for synchronized RGB/depth/TF pair"
				: "missing camera/depth image");
		return;
	}
	if (!imagesAreSynchronized(camera_msg, depth_msg)) {
		republishConfirmedCache(depth_msg->header.stamp);
		publish_timed_diagnostics("RGB/depth desynchronization");
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
		republishConfirmedCache(depth_msg->header.stamp);
		publish_timed_diagnostics("cv_bridge exception");
		return;
	}

	if (cv_ptr->image.empty() || cv_ptr->image.type() != CV_8UC1) {
		RCLCPP_ERROR(this->get_logger(), "Invalid image after conversion");
		republishConfirmedCache(depth_msg->header.stamp);
		publish_timed_diagnostics("invalid grayscale image");
		return;
	}

	// Detect lines
	std::pair<int2*,int*> line_pair;
	lines::LinePixelDetectionStats pixel_stats;
	try {
		const auto detect_start = std::chrono::steady_clock::now();
		line_pair = lines::detect_line_pixels(cv_ptr->image,
				brightness_threshold_, half_window_size_,
				sigma_threshold_, mew_threshold_,
				debug_image_write_enabled_, &pixel_stats);
		stats.cuda_detect_ms = elapsed_ms(detect_start);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Line detection failed: %s", e.what());
		republishConfirmedCache(depth_msg->header.stamp);
		publish_timed_diagnostics("line detection failure");
		return;
	}
	
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;
	stats.raw_pixels = pixel_stats.raw_pixels;
	stats.filtered_pixels = pixel_stats.filtered_pixels;
	stats.kept_components = pixel_stats.kept_components;

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Detected %d line pixels", *line_points_len);

	// Skip the 16k-pixel Int32MultiArray serialization when nobody listens.
	// /line_detection/line_pixels is a debug feed for the native HUD; the
	// costmap doesn't consume it. Saves ~3-8 ms/frame.
	if (_line_pixels_pub->get_subscription_count() > 0) {
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
		const auto temporal_start = std::chrono::steady_clock::now();
		updateTemporalConfidence({}, stamp, yaw_gated, stats);
		stats.temporal_update_ms = elapsed_ms(temporal_start);
		if (yaw_gated) {
			republishConfirmedCacheFor(
				depth_msg->header.stamp,
				motion_cache_hold_ms_,
				"missing confirmed cache during yaw gate",
				"expired motion cache during yaw gate");
		} else {
			republishConfirmedCache(depth_msg->header.stamp);
		}
		const auto debug_start = std::chrono::steady_clock::now();
		publishDebugImages(
			camera_msg, cv_ptr->image, line_points, *line_points_len,
			{}, publishedDebugPixels());
		stats.debug_publish_ms = elapsed_ms(debug_start);
		publish_timed_diagnostics(yaw_gated ? "yaw gated empty line detection" : "empty line detection");
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
		republishConfirmedCache(depth_msg->header.stamp);
		publish_timed_diagnostics("transform failure");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Bypass BFS clustering. With the shape filter stripped (curves and
	// T/L junctions aren't linear), all transformed candidates flow
	// straight into temporal voxelization. updateTemporalConfidence
	// already voxelizes at temporal_voxel_size_m (10 cm) so duplicates
	// collapse there. Eliminates the BFS pass over 16k+ points per
	// frame — was the main inline cost in line_callback that pinned
	// publish rate to ~0.8 Hz.
	std::vector<CandidatePoint> & clustered_points = transformed_points;
	stats.candidate_points = static_cast<int>(transformed_points.size());
	stats.kept_clusters = transformed_points.empty() ? 0 : 1;

	const rclcpp::Time stamp(depth_msg->header.stamp);
	const bool yaw_gated = isYawGated();
	const auto temporal_start = std::chrono::steady_clock::now();
	std::vector<Eigen::Vector3d> confirmed_points =
		updateTemporalConfidence(clustered_points, stamp, yaw_gated, stats);
	stats.temporal_update_ms = elapsed_ms(temporal_start);

	if (stats.tf_wait_failures > 0 && stats.tf_latest_fallbacks == 0 &&
		clustered_points.empty())
	{
		republishConfirmedCacheFor(
			depth_msg->header.stamp,
			motion_cache_hold_ms_,
			"missing confirmed cache after stamped TF miss",
			"expired motion cache after stamped TF miss");
	} else if (yaw_gated) {
		republishConfirmedCacheFor(
			depth_msg->header.stamp,
			motion_cache_hold_ms_,
			"missing confirmed cache during yaw gate",
			"expired motion cache during yaw gate");
	} else if (clustered_points.empty()) {
		republishConfirmedCache(depth_msg->header.stamp);
	} else if (confirmed_points.empty()) {
		republishConfirmedCache(depth_msg->header.stamp);
	} else {
		publishConfirmedOrEmpty(confirmed_points, depth_msg->header.stamp);
	}
	const auto debug_start = std::chrono::steady_clock::now();
	publishDebugImages(
		camera_msg, cv_ptr->image, line_points, *line_points_len,
		clustered_points, publishedDebugPixels());
	stats.debug_publish_ms = elapsed_ms(debug_start);
	publish_timed_diagnostics(
		(stats.tf_wait_failures > 0 && stats.tf_latest_fallbacks == 0 &&
		 clustered_points.empty()) ? "stamped TF unavailable" :
		(yaw_gated ? "yaw gated" : "updated"));

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
