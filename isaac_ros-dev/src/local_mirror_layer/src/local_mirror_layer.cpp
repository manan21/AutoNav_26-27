#include "local_mirror_layer/local_mirror_layer.hpp"

#include <algorithm>
#include <cmath>
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
  clear_topic_("/local_mirror_layer/clear"),
  track_unknown_space_(true),
  allow_decrease_(false),
  decrease_only_in_front_(false),
  decrease_angle_min_rad_(-1.2217),
  decrease_angle_max_rad_(1.2217),
  decrease_range_min_m_(0.0),
  decrease_range_max_m_(25.0),
  exclude_threshold_(1),
  min_occupied_value_to_mirror_(1),
  has_new_msg_(false),
  pending_clear_(false),
  touched_min_x_(0.0),
  touched_min_y_(0.0),
  touched_max_x_(0.0),
  touched_max_y_(0.0),
  any_touched_(false),
  clear_radius_(6.0),
  latest_robot_x_(0.0),
  latest_robot_y_(0.0),
  latest_robot_yaw_(0.0),
  have_robot_pose_(false)
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
  declareParameter("clear_topic", rclcpp::ParameterValue("/local_mirror_layer/clear"));
  declareParameter("clear_radius", rclcpp::ParameterValue(6.0));
  declareParameter("track_unknown_space", rclcpp::ParameterValue(true));
  declareParameter("allow_decrease", rclcpp::ParameterValue(false));
  declareParameter("decrease_only_in_front", rclcpp::ParameterValue(false));
  declareParameter("decrease_angle_min_rad", rclcpp::ParameterValue(-1.2217));
  declareParameter("decrease_angle_max_rad", rclcpp::ParameterValue(1.2217));
  declareParameter("decrease_range_min_m", rclcpp::ParameterValue(0.0));
  declareParameter("decrease_range_max_m", rclcpp::ParameterValue(25.0));
  declareParameter(
    "exclude_topics",
    rclcpp::ParameterValue(std::vector<std::string>()));
  declareParameter("exclude_threshold", rclcpp::ParameterValue(1));
  declareParameter("min_occupied_value_to_mirror", rclcpp::ParameterValue(1));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "source_topic", source_topic_);
  node->get_parameter(name_ + "." + "clear_topic", clear_topic_);
  node->get_parameter(name_ + "." + "clear_radius", clear_radius_);
  clear_radius_ = std::max(0.0, clear_radius_);
  node->get_parameter(name_ + "." + "track_unknown_space", track_unknown_space_);
  node->get_parameter(name_ + "." + "allow_decrease", allow_decrease_);
  node->get_parameter(name_ + "." + "decrease_only_in_front", decrease_only_in_front_);
  node->get_parameter(name_ + "." + "decrease_angle_min_rad", decrease_angle_min_rad_);
  node->get_parameter(name_ + "." + "decrease_angle_max_rad", decrease_angle_max_rad_);
  node->get_parameter(name_ + "." + "decrease_range_min_m", decrease_range_min_m_);
  node->get_parameter(name_ + "." + "decrease_range_max_m", decrease_range_max_m_);
  std::vector<std::string> configured_exclude_topics;
  node->get_parameter(name_ + "." + "exclude_topics", configured_exclude_topics);
  node->get_parameter(name_ + "." + "exclude_threshold", exclude_threshold_);
  node->get_parameter(
    name_ + "." + "min_occupied_value_to_mirror",
    min_occupied_value_to_mirror_);
  decrease_range_min_m_ = std::max(0.0, decrease_range_min_m_);
  decrease_range_max_m_ = std::max(decrease_range_min_m_, decrease_range_max_m_);
  exclude_threshold_ = std::min(100, std::max(1, exclude_threshold_));
  min_occupied_value_to_mirror_ = std::min(
    100, std::max(1, min_occupied_value_to_mirror_));
  exclude_topics_.clear();
  for (const auto & topic : configured_exclude_topics) {
    if (!topic.empty()) {
      exclude_topics_.push_back(topic);
    }
  }

  // Volatile QoS — the local costmap publishes continuously, no need
  // for transient-local replay of stale messages.
  rclcpp::QoS qos(1);
  qos.reliable();
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    source_topic_, qos,
    std::bind(&LocalMirrorLayer::mapCallback, this, std::placeholders::_1));

  latest_exclude_msgs_.resize(exclude_topics_.size());
  exclude_subs_.reserve(exclude_topics_.size());
  rclcpp::SubscriptionOptions exclude_sub_options;
  exclude_sub_options.callback_group = callback_group_;
  for (std::size_t i = 0; i < exclude_topics_.size(); ++i) {
    exclude_subs_.push_back(
      node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        exclude_topics_[i], qos,
        [this, i](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          exclusionCallback(i, msg);
        },
        exclude_sub_options));
  }

  // Clear-request topic: pressing Y on the controller publishes an
  // Empty message here. updateCosts() consumes the flag on its next
  // cycle and zeroes this layer's accumulator within the current
  // source-msg footprint, then re-stamps live obstacles in the same
  // cycle. Cells outside the local's footprint are left alone (so
  // the persistent global map further from the robot survives).
  //
  // Explicitly bind this subscription to the Layer's callback_group_
  // (passed in by Costmap2DROS during initialize()). That's the group
  // the costmap's dedicated SingleThreadedExecutor actually spins.
  // Without this, the subscription lands on the lifecycle node's
  // default callback group; debugging on hardware showed that group's
  // callbacks never fire even though `ros2 node info` lists the
  // subscription. mapCallback above happens to work without this
  // binding because nav2 happens to keep its default group serviced
  // through other paths — but the Empty subscription does not.
  rclcpp::SubscriptionOptions clear_sub_options;
  clear_sub_options.callback_group = callback_group_;
  clear_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    clear_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&LocalMirrorLayer::clearCallback, this, std::placeholders::_1),
    clear_sub_options);

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
    "LocalMirrorLayer subscribed to %s (host frame=%s, allow_decrease=%s, decrease_only_in_front=%s, exclude_topics=%zu, exclude_threshold=%d, min_occupied_value_to_mirror=%d)",
    source_topic_.c_str(),
    layered_costmap_->getGlobalFrameID().c_str(),
    allow_decrease_ ? "true" : "false",
    decrease_only_in_front_ ? "true" : "false",
    exclude_topics_.size(),
    exclude_threshold_,
    min_occupied_value_to_mirror_);
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

bool LocalMirrorLayer::decreaseAllowedAt(
  double wx, double wy,
  double robot_x, double robot_y, double robot_yaw,
  bool have_pose) const
{
  if (!decrease_only_in_front_) {
    return true;
  }
  if (!have_pose) {
    return false;
  }

  const double dx = wx - robot_x;
  const double dy = wy - robot_y;
  const double range = std::hypot(dx, dy);
  if (range < decrease_range_min_m_ || range > decrease_range_max_m_) {
    return false;
  }

  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  const double rel_x = c * dx + s * dy;
  const double rel_y = -s * dx + c * dy;
  const double angle = std::atan2(rel_y, rel_x);

  if (decrease_angle_min_rad_ <= decrease_angle_max_rad_) {
    return angle >= decrease_angle_min_rad_ && angle <= decrease_angle_max_rad_;
  }
  return angle >= decrease_angle_min_rad_ || angle <= decrease_angle_max_rad_;
}

bool LocalMirrorLayer::excludedByMask(
  double wx,
  double wy,
  const std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> & masks) const
{
  for (const auto & mask : masks) {
    if (!mask || mask->info.resolution <= 0.0 ||
      mask->info.width == 0 || mask->info.height == 0)
    {
      continue;
    }

    const double mx_f =
      (wx - mask->info.origin.position.x) / mask->info.resolution;
    const double my_f =
      (wy - mask->info.origin.position.y) / mask->info.resolution;
    if (mx_f < 0.0 || my_f < 0.0) {
      continue;
    }

    const unsigned int mx = static_cast<unsigned int>(mx_f);
    const unsigned int my = static_cast<unsigned int>(my_f);
    if (mx >= mask->info.width || my >= mask->info.height) {
      continue;
    }

    const int8_t value = mask->data[my * mask->info.width + mx];
    if (value >= exclude_threshold_) {
      return true;
    }
  }
  return false;
}

void LocalMirrorLayer::mapCallback(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mtx_);
  latest_msg_ = msg;
  has_new_msg_ = true;
}

void LocalMirrorLayer::exclusionCallback(
  std::size_t index,
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mtx_);
  if (index < latest_exclude_msgs_.size()) {
    latest_exclude_msgs_[index] = msg;
  }
}

void LocalMirrorLayer::clearCallback(
  std_msgs::msg::Empty::ConstSharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(msg_mtx_);
  pending_clear_ = true;
}

void LocalMirrorLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // Snapshot robot pose for clearCallback — same pattern as LineLayer.
  // updateBounds is the only place layered_costmap hands us the robot
  // pose in the target frame.
  {
    std::lock_guard<std::mutex> lock(msg_mtx_);
    latest_robot_x_ = robot_x;
    latest_robot_y_ = robot_y;
    latest_robot_yaw_ = robot_yaw;
    have_robot_pose_ = true;
  }
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

  auto node = node_.lock();
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg;
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> exclude_msgs;
  bool have_new;
  bool do_clear;
  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  bool have_pose = false;
  {
    std::lock_guard<std::mutex> lock(msg_mtx_);
    msg = latest_msg_;
    exclude_msgs = latest_exclude_msgs_;
    have_new = has_new_msg_;
    has_new_msg_ = false;
    do_clear = pending_clear_;
    pending_clear_ = false;
    robot_x = latest_robot_x_;
    robot_y = latest_robot_y_;
    robot_yaw = latest_robot_yaw_;
    have_pose = have_robot_pose_;
  }

  // Clear pass: if Y was pressed and we know where the robot is, zero
  // every accumulator cell within clear_radius_ of the robot in the
  // target frame. This wipes smears that have rolled out of the local
  // costmap's current 5m window but are still within easy reach of the
  // robot. Cells beyond clear_radius_ survive — persistent map farther
  // away isn't touched. After the wipe, the stamping loop below
  // re-paints whatever the local currently sees on top in the same
  // updateCosts cycle.
  if (do_clear && have_pose && costmap_ && resolution_ > 0.0) {
    const double r2 = clear_radius_ * clear_radius_;
    const int cells_radius = static_cast<int>(
      std::ceil(clear_radius_ / resolution_));
    unsigned int cx, cy;
    if (worldToMap(robot_x, robot_y, cx, cy)) {
      const int ix0 = std::max(0, static_cast<int>(cx) - cells_radius);
      const int iy0 = std::max(0, static_cast<int>(cy) - cells_radius);
      const int ix1 = std::min(static_cast<int>(size_x_) - 1,
        static_cast<int>(cx) + cells_radius);
      const int iy1 = std::min(static_cast<int>(size_y_) - 1,
        static_cast<int>(cy) + cells_radius);
      for (int j = iy0; j <= iy1; ++j) {
        for (int i = ix0; i <= ix1; ++i) {
          double wx, wy;
          mapToWorld(static_cast<unsigned int>(i),
                     static_cast<unsigned int>(j), wx, wy);
          const double dxc = wx - robot_x;
          const double dyc = wy - robot_y;
          if (dxc * dxc + dyc * dyc <= r2) {
            costmap_[j * size_x_ + i] = NO_INFORMATION;
          }
        }
      }
    }
  }

  // Stamp the latest source message into the layer's own costmap.
  // Cells already present in the layer (from previous publishes)
  // stay — they only get overwritten if the new cost is higher
  // or if a lower incoming cost passes the configured decrease gate.
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

    std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> active_masks;
    active_masks.reserve(exclude_msgs.size());
    for (const auto & mask : exclude_msgs) {
      if (!mask) {
        continue;
      }
      const std::string mask_frame = mask->header.frame_id.empty()
        ? source_frame : mask->header.frame_id;
      if (mask_frame != source_frame) {
        if (node) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("nav2_costmap_2d"),
            *node->get_clock(), 3000,
            "LocalMirrorLayer exclusion mask frame mismatch (%s vs source %s); skipping mask",
            mask_frame.c_str(), source_frame.c_str());
        }
        continue;
      }
      active_masks.push_back(mask);
    }

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
        if (node) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("nav2_costmap_2d"),
            *node->get_clock(), 3000,
            "LocalMirrorLayer TF unavailable (%s ← %s): %s — skipping mirror this cycle",
            target_frame.c_str(), source_frame.c_str(), ex.what());
        }
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
          if (src_val < 0) {
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

          if (excludedByMask(wx_src, wy_src, active_masks)) {
            costmap_[idx] = NO_INFORMATION;
            continue;
          }
          if (src_val > 0 && src_val < min_occupied_value_to_mirror_) {
            continue;
          }
          const unsigned char incoming = interpretCost(src_val);

          const unsigned char existing = costmap_[idx];
          if (existing == NO_INFORMATION) {
            if (incoming != FREE_SPACE) {
              costmap_[idx] = incoming;
            }
            continue;
          }

          if (incoming > existing) {
            costmap_[idx] = incoming;
            continue;
          }

          if (allow_decrease_ && incoming < existing &&
            decreaseAllowedAt(wx, wy, robot_x, robot_y, robot_yaw, have_pose))
          {
            costmap_[idx] = incoming;
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

  // Overwrite master where this layer has an opinion. Max-merging here
  // would block raytrace clears from propagating: a cell the local has
  // since raytrace-cleared (FREE_SPACE in the layer) can't downgrade an
  // existing LETHAL in master under max-merge, which is what produces
  // the "smearing into permanent walls" symptom. Layers further down
  // the plugin chain (line_layer, inflation_layer) still max-merge on
  // top of this, so lines re-stamp after any clears written here.
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
      master_array[midx] = layer_cost;
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
