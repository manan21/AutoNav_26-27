/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "line_layer/line_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

template class LineBuffer<std::shared_ptr<autonav_interfaces::msg::LinePoints>>;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

//#define DEBUG_
//#define DEBUG_2
//#define DEBUG_3
//#define DEBUG_4
// Leave high-volume debug logging disabled in normal runtime.
//#define DEBUG_n

// helper methods outside namespace

template <typename T>
bool within_bounds(T value, T min, T max) {

  return (value >= min) && (value <= max);
}


namespace line_layer
{

LineLayer::LineLayer()
: last_min_x_(0.0),
  last_min_y_(0.0),
  last_max_x_(1.0),
  last_max_y_(1.0),
  need_recalculation_(false),
  rolling_window_(false),
  publish_costmap_(false),
  clearing_(true),
  transform_tolerance_(0.2),
  max_message_age_ms_(750),
  observation_persistence_ms_(0),
  observation_persistence_resolution_m_(0.10),
  max_persisted_points_(12000)
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
LineLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("line_topic", rclcpp::ParameterValue("line_points"));
  declareParameter("rolling_window", rclcpp::ParameterValue(false));
  declareParameter("publish_costmap", rclcpp::ParameterValue(false));
  declareParameter("clearing", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("max_message_age_ms", rclcpp::ParameterValue(750));
  declareParameter("observation_persistence_ms", rclcpp::ParameterValue(0));
  declareParameter("observation_persistence_resolution_m", rclcpp::ParameterValue(0.10));
  declareParameter("max_persisted_points", rclcpp::ParameterValue(12000));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "line_topic", line_topic_);
  node->get_parameter(name_ + "." + "rolling_window", rolling_window_);
  node->get_parameter(name_ + "." + "publish_costmap", publish_costmap_);
  node->get_parameter(name_ + "." + "clearing", clearing_);
  node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + "." + "max_message_age_ms", max_message_age_ms_);
  node->get_parameter(name_ + "." + "observation_persistence_ms", observation_persistence_ms_);
  node->get_parameter(
    name_ + "." + "observation_persistence_resolution_m",
    observation_persistence_resolution_m_);
  node->get_parameter(name_ + "." + "max_persisted_points", max_persisted_points_);
  observation_persistence_ms_ = std::max<int64_t>(0, observation_persistence_ms_);
  observation_persistence_resolution_m_ = std::max(0.01, observation_persistence_resolution_m_);
  max_persisted_points_ = std::max(0, max_persisted_points_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  line_sub_ = node->create_subscription<autonav_interfaces::msg::LinePoints>(line_topic_, 1, 
    std::bind(&LineLayer::linePointCallback, this, std::placeholders::_1));
  
  if (publish_costmap_) {
    costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/line_costmap", 1);
  }

  

  matchSize();

  need_recalculation_ = false;
  current_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("nav_costmap_2d"), "hello from line land");
  
}
/// @brief I just accidentally did this wtf.... this is a line callback that mimics the cool other costmap plugins
/// @param message 
/// @param buffer 
void LineLayer::linePointCallback(autonav_interfaces::msg::LinePoints::ConstSharedPtr message) {

      #ifdef DEBUG_
      RCLCPP_INFO(rclcpp::get_logger("nav_costmap_2d"), "CALM LUH CALLBACK");
      #endif
      auto line = std::make_shared<autonav_interfaces::msg::LinePoints>(); 
      line->header = message->header;
      line->points = message->points;

      buffer_.buffer(line);
      current_ = false;
      need_recalculation_ = true;

}

std::optional<std::vector<geometry_msgs::msg::Vector3>> LineLayer::transformPointsToGlobalFrame(
  const autonav_interfaces::msg::LinePoints & message)
{
  auto node = node_.lock();
  if (!node) {
    return std::nullopt;
  }
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  std::vector<geometry_msgs::msg::Vector3> transformed_points;
  transformed_points.reserve(message.points.size());

  if (message.header.frame_id.empty() || message.header.frame_id == target_frame) {
    return message.points;
  }

  geometry_msgs::msg::TransformStamped transform;
  const bool use_latest_transform =
    message.header.frame_id == "map" && target_frame == "odom";
  try {
    if (use_latest_transform) {
      transform = tf_buffer_->lookupTransform(
        target_frame,
        message.header.frame_id,
        rclcpp::Time(0, 0, node->get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(transform_tolerance_));
    } else {
      transform = tf_buffer_->lookupTransform(
        target_frame,
        message.header.frame_id,
        rclcpp::Time(message.header.stamp),
        rclcpp::Duration::from_seconds(transform_tolerance_));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("nav_costmap_2d"), *node->get_clock(), 3000,
      "line_layer TF unavailable (%s <- %s): %s",
      target_frame.c_str(), message.header.frame_id.c_str(), ex.what());
    return std::nullopt;
  }

  for (const auto & point : message.points) {
    geometry_msgs::msg::PointStamped input_point;
    input_point.header = message.header;
    input_point.point.x = point.x;
    input_point.point.y = point.y;
    input_point.point.z = point.z;

    geometry_msgs::msg::PointStamped output_point;
    tf2::doTransform(input_point, output_point, transform);

    geometry_msgs::msg::Vector3 transformed;
    transformed.x = output_point.point.x;
    transformed.y = output_point.point.y;
    transformed.z = output_point.point.z;
    transformed_points.push_back(transformed);
  }

  return transformed_points;
}

bool LineLayer::hasObservationPersistence() const
{
  return clearing_ && observation_persistence_ms_ > 0 && max_persisted_points_ > 0;
}

std::uint64_t LineLayer::persistenceKey(double x, double y) const
{
  const auto qx = static_cast<std::int32_t>(
    std::llround(x / observation_persistence_resolution_m_));
  const auto qy = static_cast<std::int32_t>(
    std::llround(y / observation_persistence_resolution_m_));
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(qx)) << 32) |
         static_cast<std::uint32_t>(qy);
}

void LineLayer::rememberPersistentPoints(
  const std::vector<geometry_msgs::msg::Vector3> & points,
  const rclcpp::Time & stamp)
{
  if (!hasObservationPersistence()) {
    return;
  }

  for (const auto & point : points) {
    const auto key = persistenceKey(point.x, point.y);
    auto it = persisted_points_.find(key);
    if (it == persisted_points_.end()) {
      persisted_points_.emplace(key, PersistentPoint{point, stamp});
    } else {
      it->second = PersistentPoint{point, stamp};
    }
  }

  while (
    max_persisted_points_ > 0 &&
    persisted_points_.size() > static_cast<std::size_t>(max_persisted_points_))
  {
    auto oldest = persisted_points_.begin();
    for (auto it = persisted_points_.begin(); it != persisted_points_.end(); ++it) {
      if (it->second.stamp.nanoseconds() < oldest->second.stamp.nanoseconds()) {
        oldest = it;
      }
    }
    persisted_points_.erase(oldest);
  }
}

std::vector<geometry_msgs::msg::Vector3> LineLayer::activePersistentPoints(const rclcpp::Time & now)
{
  std::vector<geometry_msgs::msg::Vector3> points;
  if (!hasObservationPersistence()) {
    return points;
  }

  const rclcpp::Duration max_age =
    rclcpp::Duration::from_nanoseconds(observation_persistence_ms_ * 1000000LL);
  for (auto it = persisted_points_.begin(); it != persisted_points_.end(); ) {
    if ((now - it->second.stamp) > max_age) {
      it = persisted_points_.erase(it);
      continue;
    }
    points.push_back(it->second.point);
    ++it;
  }
  return points;
}

void LineLayer::stampPoints(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const std::vector<geometry_msgs::msg::Vector3> & points)
{
  for (const auto & point : points) {
    const double x = point.x;
    const double y = point.y;

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!master_grid.worldToMap(x, y, mx, my)) {
      continue;
    }

    if (
      static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }

    const int index_new = static_cast<int>(my * size_x_ + mx);
    costmap_[index_new] = LETHAL_OBSTACLE;
  }
}

void LineLayer::publishCostmap() {

  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  
  msg->header.frame_id = layered_costmap_->getGlobalFrameID();
  msg->header.stamp = node_.lock()->now();
  msg->info.width = size_x_;
  msg->info.height = size_y_;
  msg->info.origin.position.x = origin_x_;
  msg->info.origin.position.y = origin_y_;
  msg->data.resize(size_x_ * size_y_);
  for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
    unsigned char cost = costmap_[i];
    if (cost == NO_INFORMATION) {
      msg->data[i] = -1;

    }
    else {
      int8_t point = static_cast<int8_t>(cost * 100 / 254);
      msg->data[i] = point;
#ifdef DEBUG_n
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "point: %c", point);

#endif
      
    }

  }
  costmap_pub_->publish(std::move(msg));
  
}

// used in obstacle layer and voxel layer to correct bounds for the local costmap. 
void LineLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  if (!costmap_)
	  return;
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // we'll reset our maps to unknown space if appropriate
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);
  // make sure to clean up
  delete[] local_map;
}




// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
LineLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (!rolling_window_) {
    last_min_x_ = origin_x_;
    last_min_y_ = origin_y_;
    last_max_x_ = origin_x_ + getSizeInMetersX();
    last_max_y_ = origin_y_ + getSizeInMetersY();
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
    need_recalculation_ = false;
    return;
  }

  if (need_recalculation_) {
    
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

    

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    //*min_x = -std::numeric_limits<float>::max();
    //*min_y = -std::numeric_limits<float>::max();
    //*max_x = std::numeric_limits<float>::max();
    //*max_y = std::numeric_limits<float>::max();

      // Set a 20x20 meter area around the robot
    double half_size = 10.0; // 10 meters in each direction = 20x20 total
    
    *min_x = std::min(*min_x, robot_x - half_size);
    *min_y = std::min(*min_y, robot_y - half_size);
    *max_x = std::max(*max_x, robot_x + half_size);
    *max_y = std::max(*max_y, robot_y + half_size);

    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
LineLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "LineLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
LineLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }
  if (!costmap_  && !layered_costmap_->isRolling()) { 
#ifdef DEBUG_n
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "no costmap_");
#endif
	  return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():

  // below is a testament to my stupidity. Do not be like me. There is always a reason they have it set up the way they do.

  // Idgaf I'm overwriting just like they did
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  // Nav2 passes max_i/max_j as exclusive bounds.
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  auto node = node_.lock();
  if (!node) {
    return;
  }
  const rclcpp::Time now = node->now();

  auto clear_layer = [&]() {
    if (clearing_) {
      resetMaps();
      const auto persisted_points = activePersistentPoints(now);
      stampPoints(master_grid, min_i, min_j, max_i, max_j, persisted_points);
    }
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    current_ = true;
    if (publish_costmap_) {
      publishCostmap();
    }
  };

  #ifdef DEBUG_n
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "bounds: (min_x: %d), (min_y: %d), (max_x: %d), (max_y: %d)",min_i, max_i, min_j, max_j );
  #endif

  // joe was here

  // std::vector<geometry_msgs::msg::Vector3> points;
  #ifdef DEBUG_n
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "HEEEEEEEEEEEELP HEEEELP ME HEEEEEEEEEELP");
  #endif

  auto last = buffer_.read();
  if (!last ){
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger("nav2_costmap_2d"), *node->get_clock(), 2000,
      "line_layer buffer empty; waiting for line points");
    clear_layer();
    return;
  }
  auto last_msg = *last;
  if (!last_msg) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("nav2_costmap_2d"), *node->get_clock(), 2000,
      "line_layer received an empty buffered message");
    clear_layer();
    return;
  }
  if (last_msg->points.empty()) {
    clear_layer();
    return;
  }
  const rclcpp::Time message_stamp(last_msg->header.stamp, node->get_clock()->get_clock_type());
  const rclcpp::Duration max_age =
    rclcpp::Duration::from_nanoseconds(max_message_age_ms_ * 1000000LL);
  if (max_message_age_ms_ >= 0 && message_stamp.nanoseconds() > 0 && (now - message_stamp) > max_age) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("nav2_costmap_2d"), *node->get_clock(), 2000,
      "line_layer clearing stale line message age %.1f ms (limit %.1f ms)",
      (now - message_stamp).seconds() * 1000.0,
      max_age.seconds() * 1000.0);
    clear_layer();
    return;
  }

  auto transformed_points = transformPointsToGlobalFrame(*last_msg);
  if (!transformed_points) {
    clear_layer();
    return;
  }

  // In clearing mode (local costmap default), wipe this layer before
  // stamping current plus short-lived remembered marks. In non-clearing
  // mode (global costmap), skip the reset so cells accumulate across publishes.
  if (clearing_) {
    resetMaps();
  }

  const std::vector<geometry_msgs::msg::Vector3> * points = &(*transformed_points);
  std::vector<geometry_msgs::msg::Vector3> persisted_points;
  if (hasObservationPersistence()) {
    rememberPersistentPoints(*transformed_points, now);
    persisted_points = activePersistentPoints(now);
    points = &persisted_points;
  }

  #ifdef DEBUG_2
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "line point len: %zu", points->size());
  #endif


  
  stampPoints(master_grid, min_i, min_j, max_i, max_j, *points);

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  current_ = true;

  if (publish_costmap_) {
   publishCostmap();
  }
  
}




}  // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::LineLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(line_layer::LineLayer, nav2_costmap_2d::Layer)
