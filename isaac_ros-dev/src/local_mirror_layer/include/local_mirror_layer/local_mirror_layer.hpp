#ifndef LOCAL_MIRROR_LAYER_HPP_
#define LOCAL_MIRROR_LAYER_HPP_

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace local_mirror_layer
{

// Mirrors a source OccupancyGrid (typically /local_costmap/costmap)
// into the host costmap. Cells are accumulated in the layer's own grid;
// lower incoming costs may clear stored marks only when allow_decrease
// is true and the cell passes the configured robot-relative clearing
// gate. matchSize is overridden to preserve cells when the host costmap
// resizes. Optional exclusion masks let a combined source costmap omit
// cells that should enter the host through a separate layer instead.
class LocalMirrorLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  LocalMirrorLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override
  {
    // Deliberately do not wipe — see header comment for why we
    // accumulate. Subscribers can clear by service if ever needed.
    current_ = false;
  }

  void onFootprintChanged() override {}

  bool isClearable() override {return false;}

  // Override matchSize so resizes of the master costmap (triggered
  // by static_layer when /map_padded grows / shrinks) don't wipe
  // the accumulated cells. We save the existing grid, let the base
  // class resize, then translate the saved cells to their new cell
  // coordinates using world positions.
  void matchSize() override;

private:
  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void exclusionCallback(
    std::size_t index,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void clearCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
  // Map an OccupancyGrid cell value (-1 / 0 / 1-100) to a costmap_2d
  // internal cost (0 / 1-254 / 255).
  static unsigned char interpretCost(int8_t occ_val);
  bool excludedByMask(
    double wx,
    double wy,
    const std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> & masks) const;
  bool decreaseAllowedAt(
    double wx, double wy,
    double robot_x, double robot_y, double robot_yaw,
    bool have_pose) const;

  std::string source_topic_;
  std::string clear_topic_;
  bool track_unknown_space_;
  // If true, also overwrite cells when the incoming cost is lower
  // than the stored cost. The decrease_only_in_front_ gate can restrict
  // those decreases to the current sensor-clearing sector.
  bool allow_decrease_;
  bool decrease_only_in_front_;
  double decrease_angle_min_rad_;
  double decrease_angle_max_rad_;
  double decrease_range_min_m_;
  double decrease_range_max_m_;
  std::vector<std::string> exclude_topics_;
  int exclude_threshold_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> exclude_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  // Buffered most-recent message. Mutex guards swap. Same mutex also
  // protects pending_clear_, which is set by clearCallback and consumed
  // (then re-zeroed) at the start of updateCosts.
  std::mutex msg_mtx_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_msg_;
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> latest_exclude_msgs_;
  bool has_new_msg_;
  bool pending_clear_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Bounds of the most recently mirrored region in world coords —
  // expanded each updateBounds() call so the host updates the right
  // window.
  double touched_min_x_;
  double touched_min_y_;
  double touched_max_x_;
  double touched_max_y_;
  bool any_touched_;

  // Radius around the robot to zero out in costmap_ on Y press. Larger
  // than the source-msg footprint so smears accumulated outside the
  // local's current 5m window also disappear. Set via the clear_radius
  // parameter; default 6.0 m.
  double clear_radius_;
  // Robot pose captured in updateBounds (target frame) so clearCallback
  // can pick which cells to wipe without doing its own TF lookup.
  double latest_robot_x_;
  double latest_robot_y_;
  double latest_robot_yaw_;
  bool have_robot_pose_;
};

}  // namespace local_mirror_layer

#endif  // LOCAL_MIRROR_LAYER_HPP_
