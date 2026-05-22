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
#ifndef LINE_LAYER_HPP_
#define LINE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "autonav_interfaces/srv/anv_lines.hpp"
#include "autonav_interfaces/msg/line_points.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "line_layer/line_buffer.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

namespace line_layer
{

class LineLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  LineLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  // matchSize() is fired by Nav2 whenever the master global costmap
  // changes geometry (e.g., map_padder grows its bounding box). The
  // base CostmapLayer implementation resizes this layer's costmap_
  // buffer but leaves it zero-initialized -- which would briefly drop
  // every persisted line observation until the next updateCosts cycle
  // restamps from persisted_points_. The override below restamps
  // immediately, using the world-frame coordinates stored in
  // persisted_points_, so the global costmap stays translationally
  // locked to /map across resize events with no flicker gap.
  void matchSize() override;

  virtual void reset()
  {
    resetMaps();
    if (clearing_) {
      std::lock_guard<std::mutex> lock(persisted_points_mutex_);
      persisted_points_.clear();
    }
    current_ = false;
    need_recalculation_ = true;
  }

  virtual void onFootprintChanged();



  virtual bool isClearable() {return true;}

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  // Atomic because the subscription thread sets it true in linePointCallback
  // while the costmap-update thread reads and resets it in updateBounds.
  std::atomic<bool> need_recalculation_;
  bool rolling_window_;
  bool publish_costmap_;
  // If true (default, used by local costmap), resetMaps() runs every cycle and the layer reflects only the current message. If false (used in the global costmap), resetMaps() is skipped so cells accumulate forever — required for global obstacle memory when map_padder maintains a stable-sized grid.
  bool clearing_;
  double transform_tolerance_;
  int64_t max_message_age_ms_;
  int64_t observation_persistence_ms_;
  double observation_persistence_resolution_m_;
  bool clear_lines_only_in_view_;
  double line_clear_angle_min_rad_;
  double line_clear_angle_max_rad_;
  double line_clear_range_min_m_;
  double line_clear_range_max_m_;
  int max_persisted_points_;
  // Line-specific inflation, baked into stampPoints so this layer can
  // produce a different inflation radius than the global obstacle
  // inflation_layer. The plugin order in nav2_paramsv2.yaml puts
  // line_layer AFTER inflation_layer so the inflation_layer's larger
  // obstacle radius doesn't re-inflate line cells.
  double inflation_radius_;
  double cost_scaling_factor_;
  // Cells within this distance of a line are pinned to
  // INSCRIBED_INFLATED_OBSTACLE; the exponential decay starts from
  // there. Matching nav2's InflationLayer formula — without this
  // offset the raw exp(-k*r) falloff from the center makes the
  // visible halo ~⅓ of the configured inflation_radius_.
  double inscribed_radius_;
  // Cached inflation kernel: (dy, dx, cost) triplets within the
  // inflation radius. Rebuilt lazily when the master grid resolution
  // changes (matchSize). Without a cache, each stampPoints call would
  // recompute exp() for every cell × every line point.
  struct InflationOffset { int dy; int dx; unsigned char cost; };
  std::vector<InflationOffset> inflation_kernel_;
  double inflation_kernel_resolution_;
  void buildInflationKernel(double resolution);
  void updateOrigin(double new_origin_x, double new_origin_y);
  void publishCostmap();

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;

  rclcpp::Subscription<autonav_interfaces::msg::LinePoints>::SharedPtr line_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  std::string line_topic_;
  std::string costmap_topic_;

  // Debug clear: Y on the controller publishes std_msgs/Empty here.
  // On receipt this layer drops persisted_points_ within clear_radius_
  // of the robot's latest position so accumulated lines near the
  // robot disappear (useful when bad detections smear the global).
  // Points further away stay, so we don't have to re-walk the whole
  // course every time the operator wipes nearby smears.
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  std::string clear_topic_;
  double clear_radius_;
  void clearCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
  // Robot pose snapshot from the most recent updateBounds(), used by
  // clearCallback to decide which persisted points fall within
  // clear_radius_. Mutex guards both fields since clearCallback runs
  // on the subscription thread.
  std::mutex robot_pose_mutex_;
  double latest_robot_x_;
  double latest_robot_y_;
  double latest_robot_yaw_;
  bool have_robot_pose_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // turns out you can type anything you want in a comment
  // poop

  // when the wrapper sucks so you write a chiller one
  LineBuffer<std::shared_ptr<autonav_interfaces::msg::LinePoints>> buffer_;

  struct PersistentPoint
  {
    geometry_msgs::msg::Vector3 point;
    rclcpp::Time stamp;
  };
  std::unordered_map<std::uint64_t, PersistentPoint> persisted_points_;
  // Guards persisted_points_ across the subscription thread (linePointCallback
  // persists on receipt for "lines can never be lost" reliability) and the
  // costmap-update thread (updateCosts stamps from persisted_points_, also
  // retries persistence when callback TF was unavailable).
  std::mutex persisted_points_mutex_;

  void linePointCallback(autonav_interfaces::msg::LinePoints::ConstSharedPtr message);
  std::optional<std::vector<geometry_msgs::msg::Vector3>> transformPointsToGlobalFrame(
    const autonav_interfaces::msg::LinePoints & message);
  bool hasObservationPersistence() const;
  std::uint64_t persistenceKey(double x, double y) const;
  bool linePointInClearView(
    const geometry_msgs::msg::Vector3 & point,
    double robot_x,
    double robot_y,
    double robot_yaw,
    bool have_pose) const;
  void rememberPersistentPoints(
    const std::vector<geometry_msgs::msg::Vector3> & points,
    const rclcpp::Time & stamp);
  std::vector<geometry_msgs::msg::Vector3> activePersistentPoints(const rclcpp::Time & now);
  void stampPoints(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const std::vector<geometry_msgs::msg::Vector3> & points);
};

}  // namespace nav2_gradient_costmap_plugin

#endif  // LINE_LAYER_HPP_
