#include "line_detection/detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "autonav_interfaces/srv/anv_lines.hpp"
#include "autonav_interfaces/msg/line_points.hpp"

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
#include <mutex>

//#define DEBUG_2
//#define DEBUG_3
//#define DEBUG_LOG

class LineDetectorNode : public rclcpp::Node {

public:

	LineDetectorNode() : Node("lines"),
		tf_buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
		tf_listener(tf_buffer) {

		// set parameters
		this->declare_parameter("camera_topic", "rgb_gray/image_rect_gray");
		this->declare_parameter("depth_camera_topic", "rgb_gray/depth/raw");
		this->declare_parameter("camera_info_topic", "rgb_gray/info");
		this->declare_parameter("line_points_topic", "line_points");
		this->declare_parameter("enable_timer", true); 
		
		std::string camera_topic = this->get_parameter("camera_topic").as_string();
		std::string depth_camera_topic = this->get_parameter("depth_camera_topic").as_string();
		std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
		std::string line_points_topic = this->get_parameter("line_points_topic").as_string();
		this->get_parameter("enable_timer", enable_timer_);

		// subscribe to camera topics
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

		// PUBLISHERS 
		_line_pub = this->create_publisher<autonav_interfaces::msg::LinePoints>(
			line_points_topic, 1);
			
		_line_timer = this->create_wall_timer(
			std::chrono::seconds(1), 
			std::bind(&LineDetectorNode::line_callback, this));

		_line_point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
			"lines_pointcloud", 10);
			
		// create service for line detection
		_line_service = this->create_service<autonav_interfaces::srv::AnvLines>(
			"line_service",
			std::bind(&LineDetectorNode::line_service, this, std::placeholders::_1, std::placeholders::_2));
	}

private:

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_depth_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_model_sub;

	rclcpp::Service<autonav_interfaces::srv::AnvLines>::SharedPtr _line_service;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _line_point_cloud_pub; 
	rclcpp::Publisher<autonav_interfaces::msg::LinePoints>::SharedPtr _line_pub;
	rclcpp::TimerBase::SharedPtr _line_timer;

	std::mutex callback_lock;
	std::mutex depth_callback_lock;
	std::mutex camera_model_lock;
	
	sensor_msgs::msg::Image::SharedPtr latest_img;
	sensor_msgs::msg::Image::SharedPtr latest_depth_img;
	
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	image_geometry::PinholeCameraModel camera_model_;
	
	bool enable_timer_;
	bool configured_ = false;

	void line_service(
		const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
		std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response);
	
	void line_callback();

	std::vector<Eigen::Vector3d> map_transform(
		const sensor_msgs::msg::Image::SharedPtr depth_msg, 
		int2* line_points, 
		int line_points_len); 

	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

// gets camera params
void LineDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
	std::lock_guard<std::mutex> model_lock(camera_model_lock);
	
	if (!camera_model_.initialized()) {
		camera_model_.fromCameraInfo(*msg);
		configured_ = true;
		
		RCLCPP_INFO(this->get_logger(), "Camera model initialized successfully");
		
		if (enable_timer_) {
			RCLCPP_INFO(this->get_logger(), 
				"Publishing enabled on topic: %s", 
				this->get_parameter("line_points_topic").as_string().c_str());
		}
	}
}

sensor_msgs::msg::PointCloud2 createPointCloud(
	const std::vector<std::array<float, 3>>& points, 
	const std::string& frame_id) 
{
	sensor_msgs::msg::PointCloud2 pointcloud;
	pointcloud.header.frame_id = frame_id;
	pointcloud.header.stamp = rclcpp::Clock().now();
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

/**
 * Converts a list of image indices to map frame coordinates  
 */
std::vector<Eigen::Vector3d> LineDetectorNode::map_transform(
	const sensor_msgs::msg::Image::SharedPtr depth_msg, 
	int2* line_points, 
	int line_points_len) 
{
	std::vector<Eigen::Vector3d> depth_line_points;
	
	if (line_points_len == 0) {
		return depth_line_points;
	}
	
	if (!line_points || !depth_msg || depth_msg->data.empty()) {
		RCLCPP_ERROR(get_logger(), "map_transform: invalid input data");
		return depth_line_points;
	}

	// Verify encoding
	if (depth_msg->encoding != "32FC1") {
		RCLCPP_ERROR(get_logger(), "Unexpected depth encoding: %s (expected 32FC1)", 
			depth_msg->encoding.c_str());
		return depth_line_points;
	}

	const size_t row_step = depth_msg->step;
	const size_t bytes_per_pixel = sizeof(float);

	if (row_step < depth_msg->width * bytes_per_pixel) {
		RCLCPP_ERROR(get_logger(), "Depth step too small: step=%zu width=%u", 
			row_step, depth_msg->width);
		return depth_line_points;
	}

	const size_t needed = row_step * depth_msg->height;
	if (depth_msg->data.size() < needed) {
		RCLCPP_ERROR(get_logger(), "Depth data too small: have=%zu need=%zu", 
			depth_msg->data.size(), needed);
		return depth_line_points;
	}

	auto depth_ptr_u8 = depth_msg->data.data();
	std::string frame_id = depth_msg->header.frame_id;
	std::vector<std::array<float, 3>> pc_vec;

	// Lambda to safely get depth value
	auto get_depth = [&](int x, int y) -> float {
		const size_t offset = (size_t)y * row_step + (size_t)x * bytes_per_pixel;
		float d;
		std::memcpy(&d, depth_ptr_u8 + offset, sizeof(float));
		return d;
	};

	// Check if transform is available
	bool transform_available = false;
	geometry_msgs::msg::TransformStamped transform;
	
	try {
		transform = tf_buffer.lookupTransform(
			"map", 
			frame_id, 
			tf2::TimePointZero,
			std::chrono::milliseconds(50)
		);
		
		auto now = this->get_clock()->now();
		auto transform_time = rclcpp::Time(transform.header.stamp);
		auto age = (now - transform_time).seconds();
		
		if (age > 1.0) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
				"Transform is stale (%.2f sec old). Is SLAM running?", age);
		} else {
			transform_available = true;
			RCLCPP_DEBUG(get_logger(), "Transform found (age: %.3f sec)", age);
		}
		
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
			"TF not available: %s", ex.what());
	}

	int valid_depth_count = 0;
	int invalid_depth_count = 0;
	int tf_success_count = 0;

	// Process each line point
	for (int i = 0; i < line_points_len; i++) {
		// Bounds checking
		if (line_points[i].x < 0 || line_points[i].x >= (int)depth_msg->width ||
			line_points[i].y < 0 || line_points[i].y >= (int)depth_msg->height) {
			continue;
		}

		// Get depth value (in meters for ZED)
		float depth_meters = get_depth(line_points[i].x, line_points[i].y);
		
		#ifdef DEBUG_2
		if (valid_depth_count + invalid_depth_count < 5) {
			RCLCPP_INFO(get_logger(), 
				"Point[%d] at pixel (%d,%d): depth=%.3f meters", 
				i, line_points[i].x, line_points[i].y, depth_meters);
		}
		#endif
		
		// Validate depth: 0.1m to 20m range
		if (depth_meters <= 0.0f || std::isnan(depth_meters) || 
			std::isinf(depth_meters) || depth_meters < 0.1f || depth_meters > 20.0f) {
			invalid_depth_count++;
			continue;
		}
		
		valid_depth_count++;

		// Project pixel to 3D with camera model (thread-safe)
		cv::Point3d ray;
		{
			std::lock_guard<std::mutex> model_lock(camera_model_lock);
			
			if (!camera_model_.initialized()) {
				RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
					"Camera model not initialized");
				continue;
			}

			// Bounds check against camera model
			if (line_points[i].x < 0 || line_points[i].x >= (int)camera_model_.cameraInfo().width ||
				line_points[i].y < 0 || line_points[i].y >= (int)camera_model_.cameraInfo().height) {
				continue;
			}

			try {
				ray = camera_model_.projectPixelTo3dRay(
					cv::Point2d(line_points[i].x, line_points[i].y));
			} catch (const std::exception& e) {
				RCLCPP_ERROR(get_logger(), "projectPixelTo3dRay failed: %s", e.what());
				continue;
			}
		}
		
		// Scale ray by depth to get 3D point in camera frame (meters)
		float point_x = static_cast<float>(ray.x * depth_meters);
		float point_y = static_cast<float>(ray.y * depth_meters);
		float point_z = static_cast<float>(ray.z * depth_meters);
		
		// Sanity check
		if (std::isnan(point_x) || std::isnan(point_y) || std::isnan(point_z)) {
			continue;
		}

		// Add to pointcloud for visualization
		pc_vec.push_back({point_x, point_y, point_z});

		// Transform to map frame if available
		if (transform_available) {
			try {
				geometry_msgs::msg::PointStamped camera_point;
				camera_point.header = depth_msg->header;
				camera_point.point.x = point_x;
				camera_point.point.y = point_y;
				camera_point.point.z = point_z;
				
				geometry_msgs::msg::PointStamped map_point;
				tf2::doTransform(camera_point, map_point, transform);
				
				if (!std::isnan(map_point.point.x) && !std::isnan(map_point.point.y)) {
					depth_line_points.emplace_back(
						map_point.point.x, 
						map_point.point.y, 
						0.0  // Project to ground plane
					);
					tf_success_count++;
				}
				
			} catch (const std::exception& ex) {
				RCLCPP_DEBUG(get_logger(), "Transform error: %s", ex.what());
			}
		}
	}

	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
		"Depth: %d valid, %d invalid | Map: %d transformed", 
		valid_depth_count, invalid_depth_count, tf_success_count);

	// Publish pointcloud for visualization
	if (!pc_vec.empty()) {
		try {
			sensor_msgs::msg::PointCloud2 pointcloud = createPointCloud(pc_vec, frame_id);
			_line_point_cloud_pub->publish(pointcloud);
		} catch (const std::exception& e) {
			RCLCPP_ERROR_ONCE(get_logger(), "Failed to publish pointcloud: %s", e.what());
		}
	}

	return depth_line_points;
}

void LineDetectorNode::line_service(
	const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
	std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response)
{
	(void)request;

	// Get latest images thread-safely
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

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "camera message read");
	#endif

	// Convert camera image to cv::Mat
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);  
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat camera_img = cv_ptr->image;

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "camera bridge complete");
	#endif

	// Detect lines
	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(camera_img);
	int2* line_points;
	int* line_points_len;
	std::tie(line_points, line_points_len) = line_pair;

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "lines detected. points: %d", *line_points_len);
	#endif
	
	std::vector<Eigen::Vector3d> map_points = map_transform(depth_camera_msg, line_points, *line_points_len);

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "transform complete");
	#endif

	// Populate service response
	for (const auto & point: map_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		response->points.emplace_back(vec_msg);
	}

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

void LineDetectorNode::line_callback()
{
	// Check if configured
	if (!configured_){
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"Not configured - waiting for camera info");
		return;
	}
	
	if (!enable_timer_){
		return;
	}

	// Get latest images thread-safely
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
	}();
	
	if (!camera_msg) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"No camera image received yet");
		return;
	}
	
	sensor_msgs::msg::Image::SharedPtr depth_camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
	}();

	if (!depth_camera_msg) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"No depth image received yet");
		return;
	}

	RCLCPP_DEBUG(this->get_logger(), "Processing image: %s %dx%d", 
		camera_msg->encoding.c_str(), camera_msg->width, camera_msg->height);

	// Convert to grayscale
	cv_bridge::CvImagePtr cv_ptr;
	try {
		if (camera_msg->encoding == "bgra8" || camera_msg->encoding == "bgr8") {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
			cv::Mat gray_img;
			cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
			cv_ptr->image = gray_img;
			cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
		} else {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);
		}
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}
	
	cv::Mat camera_img = cv_ptr->image;
	
	if (camera_img.empty() || camera_img.type() != CV_8UC1) {
		RCLCPP_ERROR(this->get_logger(), "Invalid image after conversion");
		return;
	}

	// Detect lines
	std::pair<int2*,int*> line_pair;
	try {
		line_pair = lines::detect_line_pixels(camera_img);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Line detection failed: %s", e.what());
		return;
	}
	
	int2* line_points;
	int* line_points_len;
	std::tie(line_points, line_points_len) = line_pair;

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Detected %d line pixels", *line_points_len);
	
	if (*line_points_len == 0) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
			"No line pixels detected");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Transform to map frame
	std::vector<Eigen::Vector3d> map_points;
	try {
		map_points = map_transform(depth_camera_msg, line_points, *line_points_len);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
		delete[] line_points;
		delete line_points_len;
		return;
	}

	if (map_points.empty()) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"No valid map points after transform");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Publishing %zu line points (%.1f%% valid)",
		map_points.size(), 100.0 * map_points.size() / *line_points_len);

	// Create and publish message
	auto message = autonav_interfaces::msg::LinePoints();
	for (const auto & point: map_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		message.points.emplace_back(vec_msg);
	}

	_line_pub->publish(message);

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LineDetectorNode>());
	rclcpp::shutdown();
	return 0;
}