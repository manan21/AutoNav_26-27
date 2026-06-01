
#ifndef CUDA_CUH
#define CUDA_CUH

#include <cuda_runtime.h>
#include <npp.h>
#include <cstdint>
#include <cstddef>
#include <cstdio>


// CERIAS line-pixel kernel.
//
// Per-pixel local statistics inside a (2 * half_window + 1)^2 window
// computed from the integral images. A pixel passes the test (and gets
// emitted as a line pixel) iff it is brighter than brightness_threshold
// AND its local stddev < sigma_threshold AND local mean > mew_threshold.
// The brightness pre-gate reads the uploaded grayscale image directly on
// the GPU; no host-side cv::threshold/mask upload is needed. Defaults are
// wired through node.cpp from line_detector.yaml; not compile-time consts.
// Pass 1: classify each pixel. Writes a dense binary line mask
// (line_mask[y*width+x] = 1 for line candidates) instead of a compacted
// list, so the speckle filter (pass 2) can do neighbor lookups. Also
// increments raw_counter for diagnostics (pre-speckle pixel count).
__global__ void __cerias_kernel(const uint8_t * gray_u8,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * line_mask,
                             int * raw_counter,
                             int width, int height,
                             int half_window,
                             float brightness_threshold,
                             float sigma_threshold,
                             float mew_threshold);


extern "C" void cerias_kernel(const uint8_t * gray_u8,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * line_mask,
                             int * raw_counter,
                             int width, int height,
                             int half_window,
                             float brightness_threshold,
                             float sigma_threshold,
                             float mew_threshold);


// Pass 2: GPU speckle filter + compaction. Replaces the host-side
// cv::connectedComponentsWithStats. A mask pixel survives iff it has at
// least min_neighbors line-mask neighbors within `radius` (Chebyshev
// window) — isolated speckle has few neighbors, line pixels have many.
// Survivors are compacted into output[atomicAdd(counter)] as (x, y).
__global__ void __speckle_filter_kernel(const uint8_t * line_mask,
                             int2 * output,
                             int * counter,
                             int width, int height,
                             int radius,
                             int min_neighbors);


extern "C" void speckle_filter_kernel(const uint8_t * line_mask,
                             int2 * output,
                             int * counter,
                             int width, int height,
                             int radius,
                             int min_neighbors);

struct LineProjectionResult
{
    float target_x;
    float target_y;
    float target_z;
    float base_x;
    float base_y;
    float base_z;
    int pixel_x;
    int pixel_y;
};

struct LineProjectionStats
{
    int valid_depth = 0;
    int depth_rejects = 0;
    int depth_fill_hits = 0;
    int roi_rejects = 0;
    int geometry_rejects = 0;
    int out_of_bounds = 0;
    int transform_rejects = 0;
    int projected_count = 0;
};

extern "C" cudaError_t project_line_pixels_cuda(
    const int2 * host_line_points,
    int line_points_len,
    const uint8_t * host_depth_data,
    std::size_t depth_data_bytes,
    int depth_width,
    int depth_height,
    std::size_t depth_row_step,
    float fx,
    float fy,
    float cx,
    float cy,
    const float * target_transform_row_major,
    const float * base_transform_row_major,
    int projection_max_points,
    int roi_min_y,
    float max_depth_m,
    float base_min_x_m,
    float base_max_x_m,
    float base_max_abs_y_m,
    float ground_z_m,
    float ground_z_tolerance_m,
    int depth_fill_radius_px,
    int depth_fill_min_neighbors,
    float depth_fill_max_spread_m,
    LineProjectionResult * host_results,
    int host_results_capacity,
    LineProjectionStats * host_stats);

#endif
