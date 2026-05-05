// Pure C++ + Eigen port of the simulator-validated PCA grade detection
// pipeline (Steps 1-5 of terrain-grade-layer-plan.md). No ROS deps in
// this header so the algorithm can be unit-tested standalone.
//
// Reference implementation: lidar_sim_gui.py in
// /Users/nathanfikes/Projects/Claude-Sandbox/Lidar-Simulation/
//
// RULES.md #1: All slope math runs in the SENSOR FRAME against the
// LiDAR-derived surface normal. No IMU / world-up references.
// RULES.md #8: Whole pipeline must complete in <60 ms.

#ifndef AUTONAV_DETECTION_GRADE_DETECTOR_HPP_
#define AUTONAV_DETECTION_GRADE_DETECTOR_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace autonav_detection {

struct GradeDetectorParams {
  // Costmap geometry (sensor-frame local grid)
  float internal_resolution = 0.10f;   // fine grid cell (m)
  float grid_half_size = 8.0f;         // local grid extent (±m)

  // Grade threshold
  float traversable_max_deg = 16.7f;   // ~2x competition ramp (~8.5°)
  float pca_noise_margin_deg = 1.5f;
  float pca_max_valid_deg = 60.0f;

  // Sensor surface normal (Step 1)
  int   surface_normal_samples = 15;
  float surface_normal_radius = 0.5f;
  float surface_normal_z_window = 0.15f;
  float surface_normal_max_tilt_deg = 45.0f;

  // Ground / wall split (Step 2)
  float z_ground_band = 0.1f;
  float wall_min_height = 0.5f;
  int   min_pca_points = 6;

  // PCA classification (Step 3)
  float pca_planarity_max = 0.005f;
  int   wall_adjacent_dilation = 2;

  // Spike detection (Step 4)
  float spike_height = 0.15f;
  int   spike_min_elevated = 2;

  // DBSCAN (Step 5)
  float dbscan_eps = 0.3f;
  int   dbscan_min_samples = 3;
  int   min_cluster_size = 15;
};

struct GradeDetectorResult {
  // Obstacle points in the SAME frame as the input cloud. Caller publishes
  // these to /scan_pca_filtered_points and lets Nav2's TF handle the rest.
  std::vector<Eigen::Vector3f> obstacle_points;

  // Detected surface normal (in the algorithm's internal frame, i.e.
  // base-link-aligned). Mainly for debug/RVIZ.
  Eigen::Vector3f surface_normal{0.0f, 0.0f, 1.0f};
  bool surface_normal_valid = false;

  // Optional debug grid (for /terrain/grade_map). 8-bit signed cost
  // values: -1=unknown, 0=free, 100=lethal. Same indexing convention
  // as nav_msgs/OccupancyGrid: row-major, y increasing in row direction.
  std::vector<int8_t> grade_map;
  int   grade_map_width = 0;
  int   grade_map_height = 0;
  float grade_map_resolution = 0.0f;
  // Origin in the internal (algorithm) frame: bottom-left corner.
  float grade_map_origin_x = 0.0f;
  float grade_map_origin_y = 0.0f;
};

class GradeDetector {
 public:
  explicit GradeDetector(const GradeDetectorParams& params);

  // Set new parameters. Cheap; algorithm holds no per-frame state.
  void setParams(const GradeDetectorParams& params) { params_ = params; }
  const GradeDetectorParams& params() const { return params_; }

  // Main entry. `cloud_internal` is the input cloud already rotated into
  // the algorithm's internal frame (z = local up). The caller is responsible
  // for that pre-rotation (typically a one-shot TF lookup from the lidar
  // mount frame to base_link).
  //
  // `out.obstacle_points` is filled with points from `cloud_internal`
  // (still in the internal frame); the caller is responsible for rotating
  // them back to the publication frame.
  //
  // If `populate_grade_map` is true, `out.grade_map` is filled.
  void compute(const std::vector<Eigen::Vector3f>& cloud_internal,
               GradeDetectorResult& out,
               bool populate_grade_map = false);

 private:
  GradeDetectorParams params_;

  // Step 1: PCA on a 0.5 m disk of low-z points around the sensor footprint.
  Eigen::Vector3f computeSurfaceNormal(
      const std::vector<Eigen::Vector3f>& cloud_internal,
      bool& valid_out) const;

  // Per-cell PCA, returns slope angle in degrees (NaN if not planar / too sparse).
  float computeSlopeDeg(const std::vector<Eigen::Vector3f>& points,
                        const Eigen::Vector3f& ref_normal) const;

  // Cluster points; returns label per point (-1 = noise).
  std::vector<int> dbscan(const std::vector<Eigen::Vector3f>& points) const;

  // Bool-grid morphological dilation (4-connected, in-place semantics via copy).
  static std::vector<uint8_t> dilate(const std::vector<uint8_t>& grid,
                                     int width, int height, int iterations);
};

}  // namespace autonav_detection

#endif  // AUTONAV_DETECTION_GRADE_DETECTOR_HPP_
