/**
 * Cuda kernels for fast Line detection processing
 *
 * Tunable knobs (half_window, sigma_threshold, mew_threshold) are passed
 * in from the host as kernel arguments — they live in line_detector.yaml
 * and are read by node.cpp via declare_parameter.
 */
#include "autonav_detection/cuda.cuh"

#include <algorithm>

namespace
{

struct ProjectionDeviceBuffers
{
    int pixel_capacity = 0;
    int result_capacity = 0;
    std::size_t depth_capacity = 0;
    int2 * pixels = nullptr;
    uint8_t * depth = nullptr;
    LineProjectionResult * results = nullptr;
    float * target_transform = nullptr;
    float * base_transform = nullptr;
    int * counter = nullptr;
    LineProjectionStats * stats = nullptr;

    bool ensure(int new_pixel_capacity, std::size_t new_depth_capacity, int new_result_capacity)
    {
        if (new_pixel_capacity <= pixel_capacity &&
            new_depth_capacity <= depth_capacity &&
            new_result_capacity <= result_capacity &&
            pixels != nullptr && depth != nullptr && results != nullptr &&
            target_transform != nullptr && base_transform != nullptr &&
            counter != nullptr && stats != nullptr) {
            return true;
        }
        freeAll();
        if (new_pixel_capacity <= 0 || new_depth_capacity == 0 || new_result_capacity <= 0) {
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&pixels),
                       static_cast<std::size_t>(new_pixel_capacity) * sizeof(int2)) != cudaSuccess) {
            freeAll();
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&depth), new_depth_capacity) != cudaSuccess) {
            freeAll();
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&results),
                       static_cast<std::size_t>(new_result_capacity) * sizeof(LineProjectionResult)) != cudaSuccess) {
            freeAll();
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&target_transform), 16 * sizeof(float)) != cudaSuccess) {
            freeAll();
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&base_transform), 16 * sizeof(float)) != cudaSuccess) {
            freeAll();
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&counter), sizeof(int)) != cudaSuccess) {
            freeAll();
            return false;
        }
        if (cudaMalloc(reinterpret_cast<void **>(&stats), sizeof(LineProjectionStats)) != cudaSuccess) {
            freeAll();
            return false;
        }
        pixel_capacity = new_pixel_capacity;
        depth_capacity = new_depth_capacity;
        result_capacity = new_result_capacity;
        return true;
    }

    void freeAll()
    {
        if (pixels) { cudaFree(pixels); pixels = nullptr; }
        if (depth) { cudaFree(depth); depth = nullptr; }
        if (results) { cudaFree(results); results = nullptr; }
        if (target_transform) { cudaFree(target_transform); target_transform = nullptr; }
        if (base_transform) { cudaFree(base_transform); base_transform = nullptr; }
        if (counter) { cudaFree(counter); counter = nullptr; }
        if (stats) { cudaFree(stats); stats = nullptr; }
        pixel_capacity = 0;
        result_capacity = 0;
        depth_capacity = 0;
    }
};

ProjectionDeviceBuffers g_projection_bufs;

__device__ bool read_valid_depth_device(
    const uint8_t * depth,
    int width,
    int height,
    std::size_t row_step,
    int x,
    int y,
    float max_depth_m,
    float * depth_m)
{
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return false;
    }
    const std::size_t offset =
        static_cast<std::size_t>(y) * row_step + static_cast<std::size_t>(x) * sizeof(float);
    const float value = *reinterpret_cast<const float *>(depth + offset);
    if (!isfinite(value) || value < 0.1f || value > max_depth_m) {
        return false;
    }
    *depth_m = value;
    return true;
}

__device__ bool read_depth_with_fill_device(
    const uint8_t * depth,
    int width,
    int height,
    std::size_t row_step,
    int x,
    int y,
    float max_depth_m,
    int fill_radius,
    int fill_min_neighbors,
    float fill_max_spread_m,
    float * depth_m,
    bool * filled)
{
    *filled = false;
    if (read_valid_depth_device(depth, width, height, row_step, x, y, max_depth_m, depth_m)) {
        return true;
    }

    int count = 0;
    float sum = 0.0f;
    float min_depth = 1.0e20f;
    float max_depth = -1.0e20f;
    const int radius_sq = fill_radius * fill_radius;
    for (int dy = -fill_radius; dy <= fill_radius; ++dy) {
        for (int dx = -fill_radius; dx <= fill_radius; ++dx) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            const int dist_sq = dx * dx + dy * dy;
            if (dist_sq > radius_sq) {
                continue;
            }
            float candidate = 0.0f;
            if (!read_valid_depth_device(
                    depth, width, height, row_step, x + dx, y + dy, max_depth_m, &candidate)) {
                continue;
            }
            ++count;
            sum += candidate;
            min_depth = fminf(min_depth, candidate);
            max_depth = fmaxf(max_depth, candidate);
        }
    }
    if (count < fill_min_neighbors || (max_depth - min_depth) > fill_max_spread_m) {
        return false;
    }
    *depth_m = sum / static_cast<float>(count);
    *filled = true;
    return true;
}

__device__ float3 transform_point_device(const float * T, const float3 p)
{
    return make_float3(
        T[0] * p.x + T[1] * p.y + T[2] * p.z + T[3],
        T[4] * p.x + T[5] * p.y + T[6] * p.z + T[7],
        T[8] * p.x + T[9] * p.y + T[10] * p.z + T[11]);
}

__global__ void __project_line_pixels_kernel(
    const int2 * line_points,
    int line_points_len,
    const uint8_t * depth,
    int depth_width,
    int depth_height,
    std::size_t depth_row_step,
    float fx,
    float fy,
    float cx,
    float cy,
    const float * target_transform,
    const float * base_transform,
    int projection_count,
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
    LineProjectionResult * results,
    int results_capacity,
    int * counter,
    LineProjectionStats * stats)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= projection_count) {
        return;
    }
    const int source_idx = min(
        line_points_len - 1,
        static_cast<int>(
            (static_cast<long long>(idx) * static_cast<long long>(line_points_len)) /
            static_cast<long long>(projection_count)));
    const int2 pixel = line_points[source_idx];

    if (pixel.y < roi_min_y) {
        atomicAdd(&stats->roi_rejects, 1);
        return;
    }
    if (pixel.x < 0 || pixel.x >= depth_width || pixel.y < 0 || pixel.y >= depth_height) {
        atomicAdd(&stats->out_of_bounds, 1);
        return;
    }

    float depth_m = 0.0f;
    bool filled = false;
    if (!read_depth_with_fill_device(
            depth, depth_width, depth_height, depth_row_step, pixel.x, pixel.y,
            max_depth_m, depth_fill_radius_px, depth_fill_min_neighbors,
            depth_fill_max_spread_m, &depth_m, &filled)) {
        atomicAdd(&stats->depth_rejects, 1);
        return;
    }
    atomicAdd(&stats->valid_depth, 1);
    if (filled) {
        atomicAdd(&stats->depth_fill_hits, 1);
    }

    const float x_normalized = (static_cast<float>(pixel.x) - cx) / fx;
    const float y_normalized = (static_cast<float>(pixel.y) - cy) / fy;
    const float3 p_cam = make_float3(
        x_normalized * depth_m,
        y_normalized * depth_m,
        depth_m);
    const float3 p_target = transform_point_device(target_transform, p_cam);
    const float3 p_base = transform_point_device(base_transform, p_cam);

    if (!isfinite(p_target.x) || !isfinite(p_target.y) ||
        !isfinite(p_base.x) || !isfinite(p_base.y) || !isfinite(p_base.z)) {
        atomicAdd(&stats->transform_rejects, 1);
        return;
    }

    const bool in_geometry =
        p_base.x >= base_min_x_m &&
        p_base.x <= base_max_x_m &&
        fabsf(p_base.y) <= base_max_abs_y_m &&
        fabsf(p_base.z - ground_z_m) <= ground_z_tolerance_m;
    if (!in_geometry) {
        atomicAdd(&stats->geometry_rejects, 1);
        return;
    }

    const int output_idx = atomicAdd(counter, 1);
    if (output_idx >= results_capacity) {
        return;
    }
    LineProjectionResult out;
    out.target_x = p_target.x;
    out.target_y = p_target.y;
    out.target_z = 0.0f;
    out.base_x = p_base.x;
    out.base_y = p_base.y;
    out.base_z = p_base.z;
    out.pixel_x = pixel.x;
    out.pixel_y = pixel.y;
    results[output_idx] = out;
}

}  // namespace


// dim3 block (16,16,1)
// dim3 grid(COLS, ROWS)
__global__ void __cerias_kernel (
        float *gray_img,
        Npp32f *integral,
        Npp64f *integral_sq,
        uint8_t *brightness_mask,
        int2 *output,
        int *counter,
        int width, int height,
        int half_window,
        float sigma_threshold,
        float mew_threshold
    )
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;


    // coordinates not in brightness mask, pixel not in line
    if (x >= width || y >= height || !brightness_mask[y * width + x])
        return;

    // assemble the window
    // add 1 to account for extra row/col, but not to
    int x1 = max(0, x - half_window);
    int y1 = max(0, y - half_window);
    int x2 = min(width - 1, x + half_window) + 1;
    int y2 = min(height - 1, y + half_window) + 1;

    // get intensity std. div of pixels in the window

    // get integral image areas from window

    // y * width + x unwraps rows into 1d and adds remaining cols
    float sum_intensity = static_cast<float>(integral[y2*(width + 1) + x2] - integral[y1* (width + 1) + x2]
                        - integral[y2*(width + 1) + x1] + integral[y1*(width + 1) + x1]);

    float sum_intensity_sq = static_cast<float>(integral_sq[y2*(width+1) + x2] - integral_sq[y1*(width+1) + x2]
                        - integral_sq[y2*(width+1) + x1] + integral_sq[y1*(width+1) + x1]);


    float num_pixels = float((x2 - x1) * (y2 - y1));

    float mew = sum_intensity / num_pixels;

    float sigma = sqrt( (sum_intensity_sq - (sum_intensity * sum_intensity)/ num_pixels) / (num_pixels));

    if (sigma < sigma_threshold && mew > mew_threshold) {

        int index = atomicAdd(counter, 1);
        output[index] = make_int2(x, y);

    }

}

extern "C" void cerias_kernel(float * gray_img,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * mask,
                             int2 * output,
                             int * counter,
                             int width, int height,
                             int half_window,
                             float sigma_threshold,
                             float mew_threshold)
{


    dim3 block(16, 16);
    dim3 grid(
        (width + block.x - 1) / block.x,
        (height + block.y - 1) / block.y
    );

    __cerias_kernel<<<grid, block>>>(

        gray_img,
        integral, integral_sq,
        mask,
        output, counter,
        width, height,
        half_window,
        sigma_threshold,
        mew_threshold

    );


}

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
    LineProjectionStats * host_stats)
{
    if (!host_line_points || !host_depth_data || !target_transform_row_major ||
        !base_transform_row_major || !host_results || !host_stats ||
        line_points_len <= 0 || depth_data_bytes == 0 ||
        depth_width <= 0 || depth_height <= 0 ||
        projection_max_points <= 0 || host_results_capacity <= 0) {
        return cudaErrorInvalidValue;
    }

    const int projection_count = std::min(line_points_len, projection_max_points);
    if (!g_projection_bufs.ensure(line_points_len, depth_data_bytes, projection_count)) {
        return cudaErrorMemoryAllocation;
    }

    cudaError_t err = cudaMemcpy(
        g_projection_bufs.pixels,
        host_line_points,
        static_cast<std::size_t>(line_points_len) * sizeof(int2),
        cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemcpy(
        g_projection_bufs.depth,
        host_depth_data,
        depth_data_bytes,
        cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemset(g_projection_bufs.counter, 0, sizeof(int));
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemset(g_projection_bufs.stats, 0, sizeof(LineProjectionStats));
    if (err != cudaSuccess) {
        return err;
    }

    err = cudaMemcpy(
        g_projection_bufs.target_transform,
        target_transform_row_major,
        16 * sizeof(float),
        cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemcpy(
        g_projection_bufs.base_transform,
        base_transform_row_major,
        16 * sizeof(float),
        cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        return err;
    }

    constexpr int block_size = 256;
    const int grid_size = (projection_count + block_size - 1) / block_size;
    __project_line_pixels_kernel<<<grid_size, block_size>>>(
        g_projection_bufs.pixels,
        line_points_len,
        g_projection_bufs.depth,
        depth_width,
        depth_height,
        depth_row_step,
        fx,
        fy,
        cx,
        cy,
        g_projection_bufs.target_transform,
        g_projection_bufs.base_transform,
        projection_count,
        roi_min_y,
        max_depth_m,
        base_min_x_m,
        base_max_x_m,
        base_max_abs_y_m,
        ground_z_m,
        ground_z_tolerance_m,
        depth_fill_radius_px,
        depth_fill_min_neighbors,
        depth_fill_max_spread_m,
        g_projection_bufs.results,
        projection_count,
        g_projection_bufs.counter,
        g_projection_bufs.stats);

    err = cudaGetLastError();
    if (err == cudaSuccess) {
        err = cudaDeviceSynchronize();
    }
    if (err != cudaSuccess) {
        return err;
    }

    int device_count = 0;
    err = cudaMemcpy(&device_count, g_projection_bufs.counter, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        return err;
    }
    const int copy_count = std::min(std::min(device_count, projection_count), host_results_capacity);
    if (copy_count > 0) {
        err = cudaMemcpy(
            host_results,
            g_projection_bufs.results,
            static_cast<std::size_t>(copy_count) * sizeof(LineProjectionResult),
            cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            return err;
        }
    }
    err = cudaMemcpy(host_stats, g_projection_bufs.stats, sizeof(LineProjectionStats), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        return err;
    }
    host_stats->projected_count = copy_count;
    return cudaSuccess;
}
