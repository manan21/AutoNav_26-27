#ifndef LOCAL_MIRROR_LAYER_HPP_
#define LOCAL_MIRROR_LAYER_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace local_mirror_layer
{

// Mirrors a source OccupancyGrid (typically /local_costmap/costmap)
// into the host costmap. Cells are max-merged into the layer's own
// grid and accumulated forever — the source's clearing (FREE_SPACE
// cells) does NOT clear the layer, so obstacles that have rolled out
// of the source's window stay marked in the host. matchSize is
// overridden to preserve cells when the host costmap resizes (this is
// what makes the "global accumulates while map_padder dynamically
// resizes" pattern work).
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
  // Map an OccupancyGrid cell value (-1 / 0 / 1-100) to a costmap_2d
  // internal cost (0 / 1-254 / 255). NO_INFORMATION and FREE inputs
  // never overwrite stored cells (accumulation invariant).
  static unsigned char interpretCost(int8_t occ_val);

  std::string source_topic_;
  bool track_unknown_space_;
  // If true, also overwrite cells when the incoming cost is lower
  // than the stored cost. Default false → strictly accumulating.
  bool allow_decrease_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  // Buffered most-recent message. Mutex guards swap.
  std::mutex msg_mtx_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_msg_;
  bool has_new_msg_;

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
};

}  // namespace local_mirror_layer

#endif  // LOCAL_MIRROR_LAYER_HPP_
