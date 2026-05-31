#ifndef BREADCRUMB__FORWARD_BLOCKED_CHECK_HPP_
#define BREADCRUMB__FORWARD_BLOCKED_CHECK_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav_msgs/msg/path.hpp"

namespace breadcrumb
{

struct ForwardBlockedResult
{
  bool goal_behind;
  bool path_behind;
  bool path_valid;
  double rel_goal;
  double rel_path;
};

inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// LEGACY fixed-index variant — kept for backwards compatibility. The
// sim port (Phase 3) showed that a fixed offset into the path becomes
// wrong as soon as the robot reverses past the path's first segments
// (early waypoints end up *in front* of the reversed robot, giving a
// false "forward" reading). Prefer `computeForwardBlockedLookahead`
// below in new code.
inline ForwardBlockedResult computeForwardBlocked(
  double rx, double ry, double ryaw,
  double gx, double gy,
  const nav_msgs::msg::Path & prev_path,
  double angle_threshold,
  int path_lookahead_idx)
{
  ForwardBlockedResult out{};

  out.rel_goal = wrap_pi(std::atan2(gy - ry, gx - rx) - ryaw);
  out.goal_behind = std::abs(out.rel_goal) > angle_threshold;

  out.path_valid = prev_path.poses.size() >= 2;
  if (out.path_valid) {
    const int idx = std::min(
      static_cast<int>(prev_path.poses.size()) - 1,
      std::max(1, path_lookahead_idx));
    const double lx = prev_path.poses[idx].pose.position.x;
    const double ly = prev_path.poses[idx].pose.position.y;
    out.rel_path = wrap_pi(std::atan2(ly - ry, lx - rx) - ryaw);
    out.path_behind = std::abs(out.rel_path) > angle_threshold;
  }

  return out;
}

// Pure-pursuit-style lookahead — find the path waypoint closest to the
// robot, then scan FORWARD in path order for the first waypoint farther
// than `min_lookahead_m`. Use its bearing for the path-behind test.
//
// Why this is the right helper for the breadcrumb dispatch: during a
// reverse manoeuvre the robot moves AWAY from its plan's first
// segments. A fixed-index lookahead picks an early waypoint that's now
// in FRONT of the reversed robot — `path_behind` flips false, the BT
// thinks the path's good, exits BREADCRUMB_REVERSE one crumb in, and
// the controller immediately re-engages the same blocked carrot. The
// closest-point + forward-scan version reads the bearing of the NEXT
// usable waypoint, which stays behind the robot until the geometry
// actually clears.
inline ForwardBlockedResult computeForwardBlockedLookahead(
  double rx, double ry, double ryaw,
  double gx, double gy,
  const nav_msgs::msg::Path & prev_path,
  double angle_threshold,
  double min_lookahead_m = 0.6)
{
  ForwardBlockedResult out{};
  out.rel_goal = wrap_pi(std::atan2(gy - ry, gx - rx) - ryaw);
  out.goal_behind = std::abs(out.rel_goal) > angle_threshold;

  out.path_valid = prev_path.poses.size() >= 2;
  if (!out.path_valid) {
    return out;
  }

  // 1. Closest path index to the robot.
  size_t best_k = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < prev_path.poses.size(); ++k) {
    const auto & p = prev_path.poses[k].pose.position;
    const double dx = p.x - rx;
    const double dy = p.y - ry;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_k = k;
    }
  }

  // 2. Forward scan for first waypoint > min_lookahead_m from the robot.
  for (size_t k = best_k + 1; k < prev_path.poses.size(); ++k) {
    const auto & p = prev_path.poses[k].pose.position;
    const double d = std::hypot(p.x - rx, p.y - ry);
    if (d > min_lookahead_m) {
      out.rel_path = wrap_pi(std::atan2(p.y - ry, p.x - rx) - ryaw);
      out.path_behind = std::abs(out.rel_path) > angle_threshold;
      return out;
    }
  }
  // No forward waypoint far enough — path effectively ends here, treat
  // as not-behind (the FollowPath will hold near the last waypoint).
  return out;
}

// Convenience: signed bearing-error (rad, in (-π, π]) of the
// lookahead waypoint. Used by callers that want to drive a rotation
// command toward the path heading — e.g. GRADIENT_ESCAPE's Phase 2
// alignment step. Returns NaN if no usable lookahead waypoint exists.
inline double computeFirstSegBearingErrSigned(
  double rx, double ry, double ryaw,
  const nav_msgs::msg::Path & prev_path,
  double min_lookahead_m = 0.6)
{
  if (prev_path.poses.size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  size_t best_k = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < prev_path.poses.size(); ++k) {
    const auto & p = prev_path.poses[k].pose.position;
    const double d2 = (p.x - rx) * (p.x - rx) + (p.y - ry) * (p.y - ry);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_k = k;
    }
  }
  for (size_t k = best_k + 1; k < prev_path.poses.size(); ++k) {
    const auto & p = prev_path.poses[k].pose.position;
    const double d = std::hypot(p.x - rx, p.y - ry);
    if (d > min_lookahead_m) {
      return wrap_pi(std::atan2(p.y - ry, p.x - rx) - ryaw);
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace breadcrumb

#endif  // BREADCRUMB__FORWARD_BLOCKED_CHECK_HPP_
