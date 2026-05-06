// Algorithm implementation for grade_detector.
//
// Direct port of build_grade_costmap() in lidar_sim_gui.py. Operates
// entirely in the algorithm's internal frame (z = local up). The ROS
// wrapper is responsible for any TF transforms in/out.
//
// References:
//   - terrain-grade-layer-plan.md (Steps 1-5)
//   - /Users/nathanfikes/Projects/Claude-Sandbox/Lidar-Simulation/RULES.md
//   - lidar_sim_gui.py: surface_normal, _pca_slope_deg, build_grade_costmap

#include "autonav_detection/grade_detector.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autonav_detection {

namespace {

constexpr float kPi = 3.14159265358979323846f;

}  // namespace

GradeDetector::GradeDetector(const GradeDetectorParams& params)
    : params_(params) {}

// ───────────────────────────────────────────────────────────────────────
// Per-cell PCA slope (mirror of _pca_slope_deg in lidar_sim_gui.py)
// ───────────────────────────────────────────────────────────────────────
float GradeDetector::computeSlopeDeg(
    const std::vector<Eigen::Vector3f>& points,
    const Eigen::Vector3f& ref_normal) const {
  const int n = static_cast<int>(points.size());
  if (n < params_.min_pca_points) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Reject point sets without spread in at least 2 axes (single-ring,
  // line-like).
  Eigen::Vector3f mn = points.front();
  Eigen::Vector3f mx = points.front();
  for (const auto& p : points) {
    mn = mn.cwiseMin(p);
    mx = mx.cwiseMax(p);
  }
  std::array<float, 3> spreads = {mx.x() - mn.x(), mx.y() - mn.y(),
                                  mx.z() - mn.z()};
  std::sort(spreads.begin(), spreads.end());
  if (spreads[1] < 0.10f) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& p : points) centroid += p;
  centroid /= static_cast<float>(n);

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : points) {
    Eigen::Vector3f d = p - centroid;
    cov += d * d.transpose();
  }
  cov /= static_cast<float>(n - 1);

  // computeDirect() uses Eigen's closed-form analytical solver for 2x2
  // and 3x3 self-adjoint matrices — ~5-10× faster than the iterative
  // Jacobi method that the constructor calls by default. Plenty accurate
  // for our slope tolerance.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
  es.computeDirect(cov);
  const Eigen::Vector3f eigvals = es.eigenvalues();  // ascending

  // Reject 1D (line-like).
  if (eigvals[1] < eigvals[2] * 0.01f) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  // Reject non-planar (e.g. ramp + adjacent wall mixed in one neighborhood).
  if (eigvals[2] > 1e-12f &&
      (eigvals[0] / eigvals[2]) > params_.pca_planarity_max) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  Eigen::Vector3f normal = es.eigenvectors().col(0);
  const float cos_angle = std::abs(normal.dot(ref_normal)) /
                          (normal.norm() * ref_normal.norm());
  return std::acos(std::clamp(cos_angle, 0.0f, 1.0f)) * 180.0f / kPi;
}

// ───────────────────────────────────────────────────────────────────────
// (Old O(n²) DBSCAN deleted — replaced by an inline grid-indexed version
// inside compute(). The centroid set is bound to the algorithm's cell
// grid, so neighbor lookups become a small fixed-window scan.)
// ───────────────────────────────────────────────────────────────────────

// ───────────────────────────────────────────────────────────────────────
// Morphological dilation on a bool-as-uint8 grid (8-connected; mirrors
// scipy.ndimage.binary_dilation default structure)
// ───────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GradeDetector::dilate(const std::vector<uint8_t>& grid,
                                           int width, int height,
                                           int iterations) {
  if (iterations <= 0) return grid;
  std::vector<uint8_t> a = grid;
  std::vector<uint8_t> b(grid.size(), 0);
  for (int it = 0; it < iterations; ++it) {
    std::fill(b.begin(), b.end(), 0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (!a[y * width + x]) continue;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            b[ny * width + nx] = 1;
          }
        }
      }
    }
    std::swap(a, b);
  }
  return a;
}

// ───────────────────────────────────────────────────────────────────────
// Main entry point — Steps 2-5 of build_grade_costmap orchestrated here.
//
// On a wheeled robot driving on smooth surfaces (flat or ramps), the
// chassis is rigidly aligned with the local ground plane and the lidar
// is rigidly bolted to the chassis, so once the caller has applied the
// static URDF rotation (lidar_footprint -> base_link orientation) the
// algorithm's internal +z axis IS the local ground normal. No per-frame
// "discover the ground plane" step is needed; the simulator's
// surface_normal exists only because the sim does synthetic vertical
// ray-casts that have no analog on a real lidar. Reference normal is
// hardcoded (0,0,1).
// ───────────────────────────────────────────────────────────────────────
void GradeDetector::compute(const std::vector<Eigen::Vector3f>& cloud_input,
                            GradeDetectorResult& out,
                            bool populate_grade_map) {
  out.obstacle_points.clear();
  out.grade_map.clear();
  out.surface_normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  out.surface_normal_valid = true;

  if (cloud_input.size() < static_cast<size_t>(params_.min_pca_points)) {
    return;
  }

  // No rotation step. cloud_input is already in the algorithm's internal
  // frame (z = up) thanks to the caller's TF lookup.
  const std::vector<Eigen::Vector3f>& rotated = cloud_input;

  // ── Grid setup ──
  const float res = params_.internal_resolution;
  const float half = params_.grid_half_size;
  const int gw = std::max(1, static_cast<int>(2.0f * half / res));
  const int gh = gw;

  // cell_id → indices of points (rotated/cloud_input share the same indices).
  std::unordered_map<int, std::vector<int>> cell_idx;
  cell_idx.reserve(rotated.size() / 4);

  for (size_t i = 0; i < rotated.size(); ++i) {
    const auto& p = rotated[i];
    const int cx = static_cast<int>((p.x() + half) / res);
    const int cy = static_cast<int>((p.y() + half) / res);
    if (cx < 0 || cx >= gw || cy < 0 || cy >= gh) continue;
    cell_idx[cy * gw + cx].push_back(static_cast<int>(i));
  }

  if (cell_idx.empty()) return;

  // has_points (for dilation to find "active" 3x3 neighborhoods)
  std::vector<uint8_t> has_points(static_cast<size_t>(gw) * gh, 0);
  for (const auto& kv : cell_idx) has_points[kv.first] = 1;
  std::vector<uint8_t> has_neighbor = dilate(has_points, gw, gh, 1);

  // ── Step 2: ground / wall split, per cell, 3x3 neighborhood ──
  std::unordered_map<int, std::vector<int>> ground_cell_idx;
  std::vector<int> non_ground_idx;
  std::vector<uint8_t> wall_detected(static_cast<size_t>(gw) * gh, 0);

  // Reusable scratch buffers.
  std::vector<int> hood_idx;
  std::vector<float> zs;
  hood_idx.reserve(256);
  zs.reserve(256);

  for (int cy = 0; cy < gh; ++cy) {
    for (int cx = 0; cx < gw; ++cx) {
      if (!has_neighbor[cy * gw + cx]) continue;

      // Gather the 3x3 neighborhood's point indices.
      hood_idx.clear();
      for (int ddy = -1; ddy <= 1; ++ddy) {
        for (int ddx = -1; ddx <= 1; ++ddx) {
          const int nx = cx + ddx, ny = cy + ddy;
          if (nx < 0 || nx >= gw || ny < 0 || ny >= gh) continue;
          auto it = cell_idx.find(ny * gw + nx);
          if (it == cell_idx.end()) continue;
          hood_idx.insert(hood_idx.end(), it->second.begin(),
                          it->second.end());
        }
      }
      if (static_cast<int>(hood_idx.size()) < params_.min_pca_points) continue;

      // Sort z, scan for a wall-sized split. Real lidar layers create
      // discrete elevation rings; on a tilted surface those rings appear
      // as z-gaps between hits even though the surface is continuous.
      // We must NOT split on ring-gap artifacts (which shred a tilted
      // plate's point set into 1D slices that PCA can't classify) — only
      // on real walls. So: walk the gaps from low z up, and split at
      // the first one whose upper cluster span exceeds wall_min_height.
      // If no such gap exists, treat the whole point set as one
      // (possibly tilted) surface and let PCA assign a slope angle.
      zs.clear();
      zs.reserve(hood_idx.size());
      for (int i : hood_idx) zs.push_back(rotated[i].z());
      std::sort(zs.begin(), zs.end());

      bool has_split = false;
      float z_cut = 0.0f;
      for (size_t k = 0; k + 1 < zs.size(); ++k) {
        if (zs[k + 1] - zs[k] > params_.z_ground_band) {
          const float candidate_cut = 0.5f * (zs[k] + zs[k + 1]);
          if (zs.back() - candidate_cut <= params_.wall_min_height) {
            // Upper span is small — likely a ring-gap artifact on a
            // tilted surface, not a real wall. Keep looking for a
            // taller upper region further up the column.
            continue;
          }
          // Real wall: split, mark the cell, add upper points to the
          // non-ground obstacle candidate set.
          z_cut = candidate_cut;
          has_split = true;
          auto own_it = cell_idx.find(cy * gw + cx);
          if (own_it != cell_idx.end()) {
            for (int i : own_it->second) {
              if (rotated[i].z() > z_cut) {
                wall_detected[cy * gw + cx] = 1;
                break;
              }
            }
          }
          for (int i : hood_idx) {
            if (rotated[i].z() > z_cut) non_ground_idx.push_back(i);
          }
          break;
        }
      }

      // Ground points = own-cell points at or below the split (or all
      // own-cell points if no split happened).
      auto own_it = cell_idx.find(cy * gw + cx);
      if (own_it == cell_idx.end()) continue;
      std::vector<int>& own_ground = ground_cell_idx[cy * gw + cx];
      for (int i : own_it->second) {
        if (!has_split || rotated[i].z() <= z_cut) own_ground.push_back(i);
      }
      if (own_ground.empty()) ground_cell_idx.erase(cy * gw + cx);
    }
  }

  // ── Step 3: per-cell PCA on 3x3 ground neighborhoods ──
  const Eigen::Vector3f ref_normal(0.0f, 0.0f, 1.0f);
  std::vector<float> ms(static_cast<size_t>(gw) * gh,
                        std::numeric_limits<float>::quiet_NaN());

  std::vector<Eigen::Vector3f> hood_pts;
  hood_pts.reserve(256);

  for (const auto& kv : ground_cell_idx) {
    const int k = kv.first;
    const int cy = k / gw, cx = k % gw;
    hood_pts.clear();
    for (int ddy = -1; ddy <= 1; ++ddy) {
      for (int ddx = -1; ddx <= 1; ++ddx) {
        const int nx = cx + ddx, ny = cy + ddy;
        if (nx < 0 || nx >= gw || ny < 0 || ny >= gh) continue;
        auto it = ground_cell_idx.find(ny * gw + nx);
        if (it == ground_cell_idx.end()) continue;
        for (int i : it->second) hood_pts.push_back(rotated[i]);
      }
    }
    if (static_cast<int>(hood_pts.size()) >= params_.min_pca_points) {
      ms[k] = computeSlopeDeg(hood_pts, ref_normal);
    }
  }

  // ── Step 4: spike detection ──
  const float steep_thresh =
      params_.traversable_max_deg + params_.pca_noise_margin_deg;

  std::vector<uint8_t> vertical_obs(static_cast<size_t>(gw) * gh, 0);

  for (const auto& kv : cell_idx) {
    const int k = kv.first;
    const std::vector<int>& cell_pts = kv.second;
    if (static_cast<int>(cell_pts.size()) < params_.spike_min_elevated)
      continue;

    // Skip cells PCA already classified as traversable (with margin).
    const float m = ms[k];
    if (!std::isnan(m) && m <= steep_thresh) continue;

    // ground_z = median of bottom 30%.
    zs.clear();
    zs.reserve(cell_pts.size());
    for (int i : cell_pts) zs.push_back(rotated[i].z());
    std::sort(zs.begin(), zs.end());
    const int n_ground =
        std::max(1, static_cast<int>(zs.size()) / 3);
    const float ground_z = zs[n_ground / 2];

    int elevated = 0;
    std::vector<int> elevated_idx;
    elevated_idx.reserve(cell_pts.size());
    for (int i : cell_pts) {
      if (rotated[i].z() > ground_z + params_.spike_height) {
        ++elevated;
        elevated_idx.push_back(i);
      }
    }
    if (elevated >= params_.spike_min_elevated) {
      vertical_obs[k] = 1;
      for (int i : elevated_idx) non_ground_idx.push_back(i);
    }
  }

  // Wall + spike adjacency mask: PCA is unreliable nearby.
  std::vector<uint8_t> obstacle_or_spike(wall_detected.size(), 0);
  for (size_t i = 0; i < obstacle_or_spike.size(); ++i) {
    obstacle_or_spike[i] = (wall_detected[i] || vertical_obs[i]) ? 1 : 0;
  }
  std::vector<uint8_t> obstacle_adjacent =
      dilate(obstacle_or_spike, gw, gh, params_.wall_adjacent_dilation);

  // ── Step 5: assemble obstacle candidates and run DBSCAN ──
  std::vector<Eigen::Vector3f> candidate_pts;
  std::vector<int> candidate_src_idx;  // parallel: index into cloud_input
  candidate_pts.reserve(non_ground_idx.size() + 256);
  candidate_src_idx.reserve(non_ground_idx.size() + 256);

  for (int i : non_ground_idx) {
    candidate_pts.push_back(rotated[i]);
    candidate_src_idx.push_back(i);
  }
  // Steep ground cells (above threshold but not in wall/spike adjacency).
  for (const auto& kv : ground_cell_idx) {
    const int k = kv.first;
    const float m = ms[k];
    if (std::isnan(m)) continue;
    if (m <= steep_thresh) continue;
    if (m >= params_.pca_max_valid_deg) continue;
    if (obstacle_adjacent[k]) continue;
    for (int i : kv.second) {
      candidate_pts.push_back(rotated[i]);
      candidate_src_idx.push_back(i);
    }
  }

  // obs grid: cells whose final classification is "obstacle"
  std::vector<uint8_t> obs(static_cast<size_t>(gw) * gh, 0);

  // ── Voxel-downsample DBSCAN inputs ──
  // The raw candidate set can be 1-2k points in a populated room, and
  // DBSCAN is O(n²). Snap each candidate to its cell on the algorithm's
  // existing grid (internal_resolution m), then represent the cell by
  // one centroid. DBSCAN runs on the centroids — typically O(hundreds)
  // unique cells — for a large speedup with negligible fidelity loss.
  // Each centroid carries a "mass" (raw count) so the original
  // min_cluster_size semantics (raw point count) are preserved.
  //
  // NOTE: must NOT use std::pair<Eigen::Vector3f, int> — Eigen's default
  // ctor leaves Vector3f uninitialized, so the first point added to a
  // cell would get summed with garbage and the centroid would be wrong.
  // Use a named struct with explicit zero-init.
  struct CellAccum {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    int count = 0;
  };
  std::unordered_map<int, CellAccum> cand_cells;
  cand_cells.reserve(candidate_pts.size() / 8 + 8);
  for (const auto& p : candidate_pts) {
    const int cx = static_cast<int>((p.x() + half) / res);
    const int cy = static_cast<int>((p.y() + half) / res);
    if (cx < 0 || cx >= gw || cy < 0 || cy >= gh) continue;
    auto& slot = cand_cells[cy * gw + cx];
    slot.sum += p;
    slot.count += 1;
  }
  std::vector<Eigen::Vector3f> ds_centroids;
  std::vector<int> ds_cell_keys;
  std::vector<int> ds_mass;
  ds_centroids.reserve(cand_cells.size());
  ds_cell_keys.reserve(cand_cells.size());
  ds_mass.reserve(cand_cells.size());
  for (const auto& kv : cand_cells) {
    ds_centroids.push_back(kv.second.sum /
                           static_cast<float>(kv.second.count));
    ds_cell_keys.push_back(kv.first);
    ds_mass.push_back(kv.second.count);
  }

  if (static_cast<int>(ds_centroids.size()) >= params_.dbscan_min_samples) {
    // ── Grid-indexed DBSCAN ──
    // Centroids live on the algorithm's existing cell grid (one per
    // non-empty cell), so neighbor queries become a bounded cell-window
    // scan instead of an O(n²) all-pairs comparison. With eps = 0.30 m
    // and internal_resolution = 0.10 m, we look up to ±3 cells around
    // each centroid (a 7×7 window). Total work: O(n · w²) with w small
    // and constant.
    std::unordered_map<int, int> cell_to_centroid;
    cell_to_centroid.reserve(ds_centroids.size() * 2);
    for (size_t i = 0; i < ds_cell_keys.size(); ++i) {
      cell_to_centroid[ds_cell_keys[i]] = static_cast<int>(i);
    }
    const int eps_cells =
        std::max(1, static_cast<int>(std::ceil(params_.dbscan_eps / res)));
    const float eps2 = params_.dbscan_eps * params_.dbscan_eps;

    // Precompute neighbor lists. Self is included via the dx=dy=0 case
    // (squaredNorm = 0 ≤ eps²), preserving sklearn-compatible semantics
    // where min_samples counts the point itself.
    std::vector<std::vector<int>> neighbors(ds_centroids.size());
    for (size_t i = 0; i < ds_centroids.size(); ++i) {
      const int k = ds_cell_keys[i];
      const int cy = k / gw, cx = k % gw;
      auto& nbi = neighbors[i];
      for (int dy = -eps_cells; dy <= eps_cells; ++dy) {
        const int ny = cy + dy;
        if (ny < 0 || ny >= gh) continue;
        for (int dx = -eps_cells; dx <= eps_cells; ++dx) {
          const int nx = cx + dx;
          if (nx < 0 || nx >= gw) continue;
          auto it = cell_to_centroid.find(ny * gw + nx);
          if (it == cell_to_centroid.end()) continue;
          const int j = it->second;
          if ((ds_centroids[i] - ds_centroids[j]).squaredNorm() <= eps2) {
            nbi.push_back(j);
          }
        }
      }
    }

    // BFS over density-connected core points.
    std::vector<int> labels(ds_centroids.size(), -1);
    std::vector<uint8_t> visited(ds_centroids.size(), 0);
    int cluster_id = 0;
    for (size_t i = 0; i < ds_centroids.size(); ++i) {
      if (visited[i]) continue;
      visited[i] = 1;
      if (static_cast<int>(neighbors[i].size()) < params_.dbscan_min_samples) {
        continue;  // noise
      }
      labels[i] = cluster_id;
      std::vector<int> seeds = neighbors[i];
      for (size_t s = 0; s < seeds.size(); ++s) {
        const int q = seeds[s];
        if (!visited[q]) {
          visited[q] = 1;
          if (static_cast<int>(neighbors[q].size()) >=
              params_.dbscan_min_samples) {
            for (int nb : neighbors[q]) seeds.push_back(nb);
          }
        }
        if (labels[q] < 0) labels[q] = cluster_id;
      }
      ++cluster_id;
    }

    // Cluster mass check + mark cells.
    std::unordered_map<int, std::vector<int>> by_label;
    for (size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] < 0) continue;
      by_label[labels[i]].push_back(static_cast<int>(i));
    }
    for (const auto& kv : by_label) {
      // Cluster mass = sum of raw point counts across its voxel centroids.
      int total_mass = 0;
      for (int idx : kv.second) total_mass += ds_mass[idx];
      if (total_mass < params_.min_cluster_size) continue;
      for (int idx : kv.second) obs[ds_cell_keys[idx]] = 1;
    }
  }

  // Always mark walls + spikes, regardless of DBSCAN.
  for (size_t i = 0; i < obs.size(); ++i) {
    obs[i] = (obs[i] || wall_detected[i] || vertical_obs[i]) ? 1 : 0;
  }

  // PCA override: clear cells PCA confirmed traversable (not in spike/wall
  // bleed zone, and never clear actual spike cells).
  std::vector<uint8_t> traversable(obs.size(), 0);
  for (size_t i = 0; i < ms.size(); ++i) {
    const float m = ms[i];
    const bool good = !std::isnan(m) && m <= steep_thresh && !vertical_obs[i];
    const bool bleed = obstacle_adjacent[i] && !wall_detected[i] &&
                       !vertical_obs[i];
    traversable[i] = (good || bleed) ? 1 : 0;
  }
  for (size_t i = 0; i < obs.size(); ++i) {
    if (traversable[i] && !vertical_obs[i]) obs[i] = 0;
    if (vertical_obs[i]) obs[i] = 1;  // re-mark spikes
  }

  // ── Emit obstacle points ──
  // For every cell flagged obstacle, emit ALL points whose rotated-frame
  // cell falls into that cell (using the original-frame xyz so the
  // published cloud sits in the correct frame).
  std::unordered_set<int> emitted;
  emitted.reserve(cloud_input.size() / 4);
  for (const auto& kv : cell_idx) {
    if (!obs[kv.first]) continue;
    for (int i : kv.second) {
      if (emitted.insert(i).second) {
        out.obstacle_points.push_back(cloud_input[i]);
      }
    }
  }

  // ── Optional: build the debug grade map ──
  if (populate_grade_map) {
    out.grade_map.assign(static_cast<size_t>(gw) * gh, -1);
    out.grade_map_width = gw;
    out.grade_map_height = gh;
    out.grade_map_resolution = res;
    out.grade_map_origin_x = -half;
    out.grade_map_origin_y = -half;
    for (size_t i = 0; i < obs.size(); ++i) {
      if (!has_points[i]) continue;  // unknown
      if (obs[i]) {
        out.grade_map[i] = 100;  // lethal
      } else if (!std::isnan(ms[i])) {
        // Linearly map slope [0, traversable_max_deg] → cost [0, 99].
        const float frac = ms[i] / std::max(1.0f, params_.traversable_max_deg);
        const int cost = std::clamp(static_cast<int>(frac * 99.0f), 0, 99);
        out.grade_map[i] = static_cast<int8_t>(cost);
      } else {
        out.grade_map[i] = 0;  // observed, no slope info → free
      }
    }
  }
}

}  // namespace autonav_detection
