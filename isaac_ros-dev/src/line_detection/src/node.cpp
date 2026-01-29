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

		// Set parameters with ZED camera defaults
		this->declare_parameter("camera_topic", "/zed/zed_node/rgb/color/rect/image");
		this->declare_parameter("depth_camera_topic", "/zed/zed_node/depth/depth_registered");
		this->declare_parameter("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");
		this->declare_parameter("line_points_topic", "line_points");
		this->declare_parameter("enable_timer", true); 
		
		std::string camera_topic = this->get_parameter("camera_topic").as_string();
		std::string depth_camera_topic = this->get_parameter("depth_camera_topic").as_string();
		std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
		std::string line_points_topic = this->get_parameter("line_points_topic").as_string();
		this->get_parameter("enable_timer", enable_timer_);

		RCLCPP_INFO(this->get_logger(), "=== Line Detector Configuration ===");
		RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_camera_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Camera info: %s", camera_info_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Output topic: %s", line_points_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Timer enabled: %s", enable_timer_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "==================================");

		// subscribe to camera topics
		auto get_latest_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(callback_lock);
			latest_img = msg;
			if (!first_image_received_) {
				RCLCPP_INFO(this->get_logger(), "First RGB image received: %ux%u, encoding: %s",
					msg->width, msg->height, msg->encoding.c_str());
				first_image_received_ = true;
			}
		};
		auto get_latest_depth_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(depth_callback_lock);
			latest_depth_img = msg;
			if (!first_depth_received_) {
				RCLCPP_INFO(this->get_logger(), "First depth image received: %ux%u, encoding: %s",
					msg->width, msg->height, msg->encoding.c_str());
				first_depth_received_ = true;
			}
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
			
		RCLCPP_INFO(this->get_logger(), "LineDetectorNode initialized - waiting for camera data...");
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
	bool first_image_received_ = false;
	bool first_depth_received_ = false;

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
		
		RCLCPP_INFO(this->get_logger(), "Camera model initialized: %ux%u, fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
			msg->width, msg->height, msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
		
		if (enable_timer_) {
			RCLCPP_INFO(this->get_logger(), 
				"Publishing enabled on topic: %s", 
				this->get_parameter("line_points_topic").as_string().c_str());
		}
	}
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

/**
 * Converts a list of image indices to map frame coordinates  
 */
std::vector<Eigen::Vector3d> LineDetectorNode::map_transform(
	const sensor_msgs::msg::Image::SharedPtr depth_msg, 
	int2* line_points, 
	int line_points_len) 
{
	RCLCPP_INFO(get_logger(), "map_transform: processing %d line points", line_points_len);
	
	std::vector<Eigen::Vector3d> depth_line_points;
	
	// Early validation
	if (line_points_len <= 0) {
		RCLCPP_WARN(get_logger(), "No line points to transform");
		return depth_line_points;
	}
	
	if (!line_points) {
		RCLCPP_ERROR(get_logger(), "line_points is null!");
		return depth_line_points;
	}
	
	if (!depth_msg) {
		RCLCPP_ERROR(get_logger(), "depth_msg is null!");
		return depth_line_points;
	}
	
	if (depth_msg->data.empty()) {
		RCLCPP_ERROR(get_logger(), "depth_msg data is empty!");
		return depth_line_points;
	}

	// Check camera model initialization
	{
		std::lock_guard<std::mutex> model_lock(camera_model_lock);
		if (!camera_model_.initialized()) {
			RCLCPP_ERROR(get_logger(), "Camera model not initialized!");
			return depth_line_points;
		}
	}

	RCLCPP_INFO(get_logger(), "Depth: %ux%u, encoding: %s, step: %u",
		depth_msg->width, depth_msg->height, depth_msg->encoding.c_str(), depth_msg->step);

	// Verify encoding - ZED publishes 32FC1
	if (depth_msg->encoding != "32FC1") {
		RCLCPP_ERROR(get_logger(), "Unexpected depth encoding: %s (expected 32FC1)", 
			depth_msg->encoding.c_str());
		return depth_line_points;
	}

	const size_t row_step = depth_msg->step;
	const size_t bytes_per_pixel = sizeof(float);
	const uint8_t* depth_ptr_u8 = depth_msg->data.data();
	std::string frame_id = depth_msg->header.frame_id;
	std::vector<std::array<float, 3>> pc_vec;
	pc_vec.reserve(std::min(line_points_len, 10000));  // Cap reserve size

	RCLCPP_INFO(get_logger(), "CHECKPOINT 1: Starting depth access setup");

	// Lambda to safely get depth value
	auto get_depth = [&](int x, int y) -> float {
		const size_t offset = (size_t)y * row_step + (size_t)x * bytes_per_pixel;
		if (offset + sizeof(float) > depth_msg->data.size()) {
			return -1.0f;
		}
		float d;
		std::memcpy(&d, depth_ptr_u8 + offset, sizeof(float));
		return d;
	};

	RCLCPP_INFO(get_logger(), "CHECKPOINT 2: Lambda created, checking transform");

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
		transform_available = true;
		RCLCPP_INFO(get_logger(), "Transform available: map <- %s", frame_id.c_str());
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN(get_logger(), "TF not available (map <- %s): %s", frame_id.c_str(), ex.what());
	}

	RCLCPP_INFO(get_logger(), "CHECKPOINT 3: Starting point loop (transform_available=%d)", transform_available);

	int valid_count = 0;
	int invalid_depth = 0;
	int out_of_bounds = 0;
	int projection_fail = 0;
	int tf_success = 0;

	// Process each line point
	for (int i = 0; i < line_points_len; i++) {
		if (i % 500 == 0) {
			RCLCPP_INFO(get_logger(), "Processing point %d/%d", i, line_points_len);
		}
		
		try {
			// Bounds checking
			if (line_points[i].x < 0 || line_points[i].x >= (int)depth_msg->width ||
				line_points[i].y < 0 || line_points[i].y >= (int)depth_msg->height) {
				out_of_bounds++;
				continue;
			}

			// Get depth value (ZED outputs meters)
			float depth_m = get_depth(line_points[i].x, line_points[i].y);
			
			// Validate depth: 0.1m to 20m range
			if (depth_m < 0.1f || depth_m > 20.0f || std::isnan(depth_m) || std::isinf(depth_m)) {
				invalid_depth++;
				continue;
			}
			
			valid_count++;

			// Project pixel to 3D with camera model (thread-safe)
			cv::Point3d ray;
			bool success = false;
			
			{
				std::lock_guard<std::mutex> model_lock(camera_model_lock);
				
				if (!camera_model_.initialized()) {
					RCLCPP_ERROR(get_logger(), "Camera model became uninitialized!");
					break;
				}

				try {
					ray = camera_model_.projectPixelTo3dRay(
						cv::Point2d(line_points[i].x, line_points[i].y));
					success = true;
				} catch (const std::exception& e) {
					projection_fail++;
					if (projection_fail == 1) {
						RCLCPP_ERROR(get_logger(), "projectPixelTo3dRay failed: %s", e.what());
					}
					continue;
				}
			}
			
			if (!success) continue;
			
			// Scale ray by depth to get 3D point in camera frame
			float px = static_cast<float>(ray.x * depth_m);
			float py = static_cast<float>(ray.y * depth_m);
			float pz = static_cast<float>(ray.z * depth_m);
			
			if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) {
				continue;
			}

			// Add to pointcloud for visualization - be careful here
			try {
				std::array<float, 3> pt = {px, py, pz};
				pc_vec.push_back(pt);
			} catch (const std::exception& e) {
				RCLCPP_ERROR(get_logger(), "Failed to add to pc_vec: %s", e.what());
				break;
			}

			// Transform to map frame if available
			if (transform_available) {
				try {
					geometry_msgs::msg::PointStamped camera_point;
					camera_point.header = depth_msg->header;
					camera_point.point.x = static_cast<double>(px);
					camera_point.point.y = static_cast<double>(py);
					camera_point.point.z = static_cast<double>(pz);
					
					geometry_msgs::msg::PointStamped map_point;
					tf2::doTransform(camera_point, map_point, transform);
					
					if (!std::isnan(map_point.point.x) && !std::isnan(map_point.point.y)) {
						depth_line_points.emplace_back(
							map_point.point.x, 
							map_point.point.y, 
							0.0  // Project to ground plane
						);
						tf_success++;
					}
				} catch (const std::exception& ex) {
					if (tf_success == 0) {
						RCLCPP_ERROR(get_logger(), "First TF transform error: %s", ex.what());
					}
				}
			}
		} catch (const std::exception& e) {
			RCLCPP_ERROR(get_logger(), "Exception at point %d: %s", i, e.what());
			break;
		}
	}

	RCLCPP_INFO(get_logger(), "CHECKPOINT 4: Loop complete");
	RCLCPP_INFO(get_logger(), "Stats: %d valid, %d invalid_depth, %d out_of_bounds, %d projection_fail, %d->map",
		valid_count, invalid_depth, out_of_bounds, projection_fail, tf_success);

	// Publish pointcloud for visualization
	RCLCPP_INFO(get_logger(), "CHECKPOINT 5: Creating pointcloud with %zu points", pc_vec.size());
	
	if (!pc_vec.empty()) {
		try {
			RCLCPP_INFO(get_logger(), "CHECKPOINT 5a: About to call createPointCloud");
			sensor_msgs::msg::PointCloud2 pc = createPointCloud(pc_vec, frame_id, depth_msg->header.stamp);
			RCLCPP_INFO(get_logger(), "CHECKPOINT 5b: Pointcloud created, about to publish");
			_line_point_cloud_pub->publish(pc);
			RCLCPP_INFO(get_logger(), "CHECKPOINT 5c: Pointcloud published with %zu points", pc_vec.size());
		} catch (const std::exception& e) {
			RCLCPP_ERROR(get_logger(), "Pointcloud publish failed: %s", e.what());
		}
	}

	RCLCPP_INFO(get_logger(), "CHECKPOINT 6: Returning %zu map points", depth_line_points.size());
	return depth_line_points;
}

void LineDetectorNode::line_service(
	const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
	std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response)
{
	(void)request;
	
	RCLCPP_INFO(this->get_logger(), "Line service called");

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
	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(cv_ptr->image);
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;

	RCLCPP_INFO(this->get_logger(), "Detected %d line pixels", *line_points_len);
	
	std::vector<Eigen::Vector3d> map_points = map_transform(depth_camera_msg, line_points, *line_points_len);

	// Populate response
	for (const auto & point: map_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		response->points.emplace_back(vec_msg);
	}

	RCLCPP_INFO(this->get_logger(), "Service returning %zu points", response->points.size());

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

void LineDetectorNode::line_callback()
{
	// Check prerequisites
	if (!configured_){
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"Not configured - waiting for camera info");
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

	if (!camera_msg) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"No RGB image received");
		return;
	}
	
	if (!depth_msg) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"No depth image received");
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
		return;
	}
	
	if (cv_ptr->image.empty() || cv_ptr->image.type() != CV_8UC1) {
		RCLCPP_ERROR(this->get_logger(), "Invalid image after conversion");
		return;
	}

	// Detect lines
	std::pair<int2*,int*> line_pair;
	try {
		line_pair = lines::detect_line_pixels(cv_ptr->image);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Line detection failed: %s", e.what());
		return;
	}
	
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Detected %d line pixels", *line_points_len);
	
	if (*line_points_len == 0) {
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Transform to map frame
	std::vector<Eigen::Vector3d> map_points;
	try {
		map_points = map_transform(depth_msg, line_points, *line_points_len);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Publish
	if (!map_points.empty()) {
		auto message = autonav_interfaces::msg::LinePoints();
		for (const auto & point: map_points) {
			geometry_msgs::msg::Vector3 vec_msg;
			vec_msg.x = point.x();
			vec_msg.y = point.y();
			vec_msg.z = point.z();
			message.points.emplace_back(vec_msg);
		}

		_line_pub->publish(message);
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"Published %zu line points", message.points.size());
	}

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