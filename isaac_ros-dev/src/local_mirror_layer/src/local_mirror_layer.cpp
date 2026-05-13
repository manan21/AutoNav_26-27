#include "local_mirror_layer/local_mirror_layer.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"

namespace local_mirror_layer
{

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

LocalMirrorLayer::LocalMirrorLayer()
: source_topic_("/local_costmap/costmap"),
  track_unknown_space_(true),
  allow_decrease_(false),
  has_new_msg_(false),
  touched_min_x_(0.0),
  touched_min_y_(0.0),
  touched_max_x_(0.0),
  touched_max_y_(0.0),
  any_touched_(false)
{
}

void LocalMirrorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("source_topic", rclcpp::ParameterValue("/local_costmap/costmap"));
  declareParameter("track_unknown_space", rclcpp::ParameterValue(true));
  declareParameter("allow_decrease", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "source_topic", source_topic_);
  node->get_parameter(name_ + "." + "track_unknown_space", track_unknown_space_);
  node->get_parameter(name_ + "." + "allow_decrease", allow_decrease_);

  // Volatile QoS — the local costmap publishes continuously, no need
  // for transient-local replay of stale messages.
  rclcpp::QoS qos(1);
  qos.reliable();
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    source_topic_, qos,
    std::bind(&LocalMirrorLayer::mapCallback, this, std::placeholders::_1));

  // TF is needed when the source costmap publishes in a different
  // frame than the layered costmap (e.g. local in odom, global in
  // map). With the project's static map↔odom identity TF the lookup
  // returns identity, but routing through TF keeps the layer
  // correct if that assumption ever changes.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Set default_value_ before matchSize() so the freshly-resized
  // costmap_ is filled with NO_INFORMATION (not FREE), which is what
  // we mean by "the layer has no opinion about this cell yet".
  default_value_ = NO_INFORMATION;
  matchSize();
  current_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("nav2_costmap_2d"),
    "LocalMirrorLayer subscribed to %s (host frame=%s)",
    source_topic_.c_str(),
    layered_costmap_->getGlobalFrameID().c_str());
}

unsigned char LocalMirrorLayer::interpretCost(int8_t occ_val)
{
  if (occ_val < 0) {
    return NO_INFORMATION;
  }
  if (occ_val == 0) {
    return FREE_SPACE;
  }
  if (occ_val >= 100) {
    return LETHAL_OBSTACLE;
  }
  // Map 1..99 → 1..253 with the same scaling Costmap2D uses for
  // OccupancyGrid round-trip: lethal stays lethal, inscribed stays
  // inscribed (99 → 253 ≈ INSCRIBED_INFLATED_OBSTACLE - 1).
  return static_cast<unsigned char>(
    1 + (static_cast<int>(occ_val) * 252) / 99);
}

void LocalMirrorLayer::mapCallback(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mtx_);
  latest_msg_ = msg;
  has_new_msg_ = true;
}

void LocalMirrorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(msg_mtx_);
    msg = latest_msg_;
  }

  if (msg) {
    // Expand the update window to cover the source costmap's
    // footprint so updateCosts gets a chance to stamp every cell that
    // came in. This is one source's bounds + whatever cells we've
    // accumulated outside that window (they don't need updating
    // because they're already in the layer's grid).
    const double src_min_x = msg->info.origin.position.x;
    const double src_min_y = msg->info.origin.position.y;
    const double src_max_x =
      src_min_x + msg->info.width * msg->info.resolution;
    const double src_max_y =
      src_min_y + msg->info.height * msg->info.resolution;

    *min_x = std::min(*min_x, src_min_x);
    *min_y = std::min(*min_y, src_min_y);
    *max_x = std::max(*max_x, src_max_x);
    *max_y = std::max(*max_y, src_max_y);

    // Also ensure the persistent footprint of previously-stamped
    // cells gets re-applied to master on every cycle.
    if (any_touched_) {
      *min_x = std::min(*min_x, touched_min_x_);
      *min_y = std::min(*min_y, touched_min_y_);
      *max_x = std::max(*max_x, touched_max_x_);
      *max_y = std::max(*max_y, touched_max_y_);
    }
  } else if (any_touched_) {
    *min_x = std::min(*min_x, touched_min_x_);
    *min_y = std::min(*min_y, touched_min_y_);
    *max_x = std::max(*max_x, touched_max_x_);
    *max_y = std::max(*max_y, touched_max_y_);
  }
}

void LocalMirrorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  if (!costmap_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg;
  bool have_new;
  {
    std::lock_guard<std::mutex> lock(msg_mtx_);
    msg = latest_msg_;
    have_new = has_new_msg_;
    has_new_msg_ = false;
  }

  // Stamp the latest source message into the layer's own costmap.
  // Cells already present in the layer (from previous publishes)
  // stay — they only get overwritten if the new cost is higher
  // (or, if allow_decrease_, by any non-NO_INFORMATION value).
  if (msg && have_new) {
    const unsigned int src_w = msg->info.width;
    const unsigned int src_h = msg->info.height;
    const double src_res = msg->info.resolution;
    const double src_ox = msg->info.origin.position.x;
    const double src_oy = msg->info.origin.position.y;

    // Resolve source frame → layered-costmap frame once per message.
    // For our project this is odom → map and the static identity TF
    // makes it the identity transform; but we route through tf2 so
    // any future drift in map↔odom is handled correctly.
    const std::string target_frame = layered_costmap_->getGlobalFrameID();
    const std::string source_frame = msg->header.frame_id.empty()
      ? target_frame : msg->header.frame_id;

    double dx = 0.0;
    double dy = 0.0;
    double cos_t = 1.0;
    double sin_t = 0.0;
    if (source_frame != target_frame && tf_buffer_) {
      try {
        // Latest available — the source costmap and the layered
        // costmap update at different rates, and we want the most
        // current frame relationship, not the source message's
        // historical one.
        auto tf = tf_buffer_->lookupTransform(
          target_frame, source_frame, tf2::TimePointZero);
        dx = tf.transform.translation.x;
        dy = tf.transform.translation.y;
        const double qz = tf.transform.rotation.z;
        const double qw = tf.transform.rotation.w;
        const double yaw = 2.0 * std::atan2(qz, qw);
        cos_t = std::cos(yaw);
        sin_t = std::sin(yaw);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("nav2_costmap_2d"),
          *node_.lock()->get_clock(), 3000,
          "LocalMirrorLayer TF unavailable (%s ← %s): %s — skipping mirror this cycle",
          target_frame.c_str(), source_frame.c_str(), ex.what());
        // Don't return: still apply the persistent layer to master.
        msg.reset();
      }
    }

    if (msg) {
      // Layer's resolution is the host's resolution (set by matchSize).
      // If src_res differs we still proceed — each source cell maps to
      // whatever host cell its center falls into.
      for (unsigned int sy = 0; sy < src_h; ++sy) {
        const double wy_src = src_oy + (sy + 0.5) * src_res;
        for (unsigned int sx = 0; sx < src_w; ++sx) {
          const int8_t src_val = msg->data[sy * src_w + sx];
          const unsigned char incoming = interpretCost(src_val);

          // NO_INFORMATION input never overrides — "no opinion" from
          // source means we keep whatever we already had.
          if (incoming == NO_INFORMATION) {
            continue;
          }
          // FREE input requires allow_decrease=true to override.
          // Otherwise we're in strict accumulator mode and FREE
          // cells from the source can't clear stored marks.
          if (incoming == FREE_SPACE && !allow_decrease_) {
            continue;
          }

          const double wx_src = src_ox + (sx + 0.5) * src_res;
          // Apply 2-D rigid transform: rotate then translate.
          const double wx = cos_t * wx_src - sin_t * wy_src + dx;
          const double wy = sin_t * wx_src + cos_t * wy_src + dy;
          unsigned int mx, my;
          if (!worldToMap(wx, wy, mx, my)) {
            continue;
          }
          const unsigned int idx = my * size_x_ + mx;
          if (allow_decrease_) {
            // Within source window: mirror exactly, including FREE
            // overriding stored marks. The local's own raytracing /
            // temporal handling has already filtered transient
            // obstacles within the source window, so what we receive
            // is the trusted ground truth at this moment.
            costmap_[idx] = incoming;
          } else {
            const unsigned char existing = costmap_[idx];
            if (existing == NO_INFORMATION || incoming > existing) {
              costmap_[idx] = incoming;
            }
          }
        }
      }
    }

    // Track touched bounds in the TARGET frame (where the master
    // lives) so updateBounds re-applies our cells to master in the
    // right window every cycle. Bounding box of the four source
    // footprint corners transformed into target frame.
    if (msg) {
      const double corners_sx[4] = {src_ox, src_ox + src_w * src_res,
        src_ox, src_ox + src_w * src_res};
      const double corners_sy[4] = {src_oy, src_oy,
        src_oy + src_h * src_res, src_oy + src_h * src_res};
      double tminx = 0.0, tminy = 0.0, tmaxx = 0.0, tmaxy = 0.0;
      for (int k = 0; k < 4; ++k) {
        const double tx = cos_t * corners_sx[k] - sin_t * corners_sy[k] + dx;
        const double ty = sin_t * corners_sx[k] + cos_t * corners_sy[k] + dy;
        if (k == 0) {
          tminx = tmaxx = tx;
          tminy = tmaxy = ty;
        } else {
          tminx = std::min(tminx, tx);
          tmaxx = std::max(tmaxx, tx);
          tminy = std::min(tminy, ty);
          tmaxy = std::max(tmaxy, ty);
        }
      }
      if (!any_touched_) {
        touched_min_x_ = tminx;
        touched_min_y_ = tminy;
        touched_max_x_ = tmaxx;
        touched_max_y_ = tmaxy;
        any_touched_ = true;
      } else {
        touched_min_x_ = std::min(touched_min_x_, tminx);
        touched_min_y_ = std::min(touched_min_y_, tminy);
        touched_max_x_ = std::max(touched_max_x_, tmaxx);
        touched_max_y_ = std::max(touched_max_y_, tmaxy);
      }
    }
  }

  // Push layer cells into the master with max-merge.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);

  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int master_size_x = master_grid.getSizeInCellsX();
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const unsigned int idx = j * size_x_ + i;
      const unsigned char layer_cost = costmap_[idx];
      if (layer_cost == NO_INFORMATION) {
        continue;
      }
      const unsigned int midx = j * master_size_x + i;
      const unsigned char m = master_array[midx];
      if (m == NO_INFORMATION || layer_cost > m) {
        master_array[midx] = layer_cost;
      }
    }
  }

  current_ = true;
}

void LocalMirrorLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  const unsigned int new_size_x = master->getSizeInCellsX();
  const unsigned int new_size_y = master->getSizeInCellsY();
  const double new_res = master->getResolution();
  const double new_ox = master->getOriginX();
  const double new_oy = master->getOriginY();

  // First-time init — no prior costmap_, just resize.
  if (!costmap_) {
    resizeMap(new_size_x, new_size_y, new_res, new_ox, new_oy);
    return;
  }

  // No-op if nothing changed.
  if (new_size_x == size_x_ && new_size_y == size_y_ &&
    std::abs(new_res - resolution_) < 1e-6 &&
    std::abs(new_ox - origin_x_) < 1e-6 &&
    std::abs(new_oy - origin_y_) < 1e-6)
  {
    return;
  }

  // Snapshot existing state.
  const unsigned int old_size_x = size_x_;
  const unsigned int old_size_y = size_y_;
  const double old_res = resolution_;
  const double old_ox = origin_x_;
  const double old_oy = origin_y_;

  std::vector<unsigned char> saved(
    static_cast<size_t>(old_size_x) * old_size_y);
  std::memcpy(saved.data(), costmap_,
    static_cast<size_t>(old_size_x) * old_size_y);

  // Resize wipes costmap_ via resetMaps.
  resizeMap(new_size_x, new_size_y, new_res, new_ox, new_oy);

  // Replay every non-empty cell at its world-aligned new position.
  for (unsigned int oy = 0; oy < old_size_y; ++oy) {
    const double wy = old_oy + (oy + 0.5) * old_res;
    for (unsigned int ox = 0; ox < old_size_x; ++ox) {
      const unsigned char cost = saved[oy * old_size_x + ox];
      if (cost == NO_INFORMATION || cost == FREE_SPACE) {
        continue;
      }
      const double wx = old_ox + (ox + 0.5) * old_res;
      unsigned int nmx, nmy;
      if (!worldToMap(wx, wy, nmx, nmy)) {
        continue;
      }
      const unsigned int idx = nmy * size_x_ + nmx;
      if (cost > costmap_[idx]) {
        costmap_[idx] = cost;
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("nav2_costmap_2d"),
    "LocalMirrorLayer matchSize: %ux%u -> %ux%u (preserved cells)",
    old_size_x, old_size_y, new_size_x, new_size_y);
}

}  // namespace local_mirror_layer

PLUGINLIB_EXPORT_CLASS(local_mirror_layer::LocalMirrorLayer, nav2_costmap_2d::Layer)
