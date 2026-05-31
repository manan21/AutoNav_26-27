#include "autonav_detection/detection.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

constexpr int kMinLineComponentPixels = 20;
constexpr float kMinLineMajorAxisPx = 12.0F;
// Image-space CC aspect gate. Was 4.0 which rejected T- and L-junctions:
// when both arms of a T are bright and connected, the bounding rect of
// all the pixels is roughly square (aspect ~2), failing the gate and
// dropping the entire line into 0-1 trivial "kept components" with very
// few of the original 16k+ bright pixels surviving. 2.0 admits T/L
// junctions while still rejecting blob speckles (which are aspect ~1).
// Round bright objects (cones, dots) are intentionally NOT this
// detector's job — those are handled by the lidar/grade obstacle path.
constexpr float kMinLineAspectRatio = 2.0F;

// Persistent CUDA device buffers reused across every detect_line_pixels
// call. Previously each frame allocated and freed all seven buffers
// (input image, mask, output, counter, integral inputs/outputs) — on
// Jetson that's 2–5 ms/frame just in driver-serialized alloc/free
// overhead. Re-allocation only happens when image dimensions change,
// which in practice is "once on the first frame, never again". NOT
// thread-safe; the detection node serializes calls on its ROS executor.
//
// Leak-at-exit is intentional: by the time main() returns the CUDA
// context is torn down and cudaFree would just emit a warning.
struct LineDeviceBuffers
{
    int     width      = 0;
    int     height     = 0;
    Npp8u   *u8_input    = nullptr;
    Npp32f  *integral    = nullptr;
    Npp64f  *integral_sq = nullptr;
    int2    *output      = nullptr;
    int     *counter     = nullptr;

    bool ensure(int new_w, int new_h);
    void freeAll();
};

bool LineDeviceBuffers::ensure(int new_w, int new_h)
{
    if (new_w == width && new_h == height && u8_input != nullptr) {
        return true;
    }
    freeAll();
    const size_t total = static_cast<size_t>(new_w) * new_h;
    const size_t int_cells =
        static_cast<size_t>(new_h + 1) * static_cast<size_t>(new_w + 1);

    if (cudaMalloc(reinterpret_cast<void**>(&u8_input),
                   total) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&integral),
                   int_cells * sizeof(Npp32f)) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&integral_sq),
                   int_cells * sizeof(Npp64f)) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&output),
                   total * sizeof(int2)) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&counter),
                   sizeof(int)) != cudaSuccess) { freeAll(); return false; }

    // Zero the integral buffers once, here. nppiSqrIntegral rewrites the
    // full interior every frame and the zero top-row/left-column border
    // never changes, so the previous per-frame memsets of these (~6 MB
    // combined at 540x960) were redundant work on the hot path.
    if (cudaMemset(integral, 0, int_cells * sizeof(Npp32f)) != cudaSuccess) {
        freeAll();
        return false;
    }
    if (cudaMemset(integral_sq, 0, int_cells * sizeof(Npp64f)) != cudaSuccess) {
        freeAll();
        return false;
    }

    width = new_w;
    height = new_h;
    return true;
}

void LineDeviceBuffers::freeAll()
{
    if (u8_input)    { cudaFree(u8_input);    u8_input    = nullptr; }
    if (integral)    { cudaFree(integral);    integral    = nullptr; }
    if (integral_sq) { cudaFree(integral_sq); integral_sq = nullptr; }
    if (output)      { cudaFree(output);      output      = nullptr; }
    if (counter)     { cudaFree(counter);     counter     = nullptr; }
    width = 0;
    height = 0;
}

LineDeviceBuffers g_line_bufs;

std::pair<int2 *, int *> filter_line_components(
    const int2 *points,
    int count,
    int width,
    int height,
    int roi_min_y,
    int * kept_component_count)
{
    int *filtered_count = new int(0);
    int2 *filtered_points = new int2[1];

    if (!points || count <= 0 || width <= 0 || height <= 0) {
        return std::make_pair(filtered_points, filtered_count);
    }

    // Connected components only need to run over the ROI band. The top
    // rows are zeroed before detection so the kernel never emits pixels
    // there; scanning them in CC is wasted work (~45% of the image at the
    // default roi_min_y_fraction). Restrict the labeling to
    // [roi_y, height) and offset y coordinates accordingly.
    const int roi_y = std::clamp(roi_min_y, 0, height - 1);
    const int roi_h = height - roi_y;

    // Persist the component_mask between frames; only allocate on size
    // change. cv::Mat::zeros allocates fresh memory every frame; setTo(0)
    // reuses the buffer. The mask is sized to the ROI band only.
    static cv::Mat component_mask;
    if (component_mask.rows != roi_h || component_mask.cols != width ||
        component_mask.type() != CV_8UC1) {
        component_mask = cv::Mat::zeros(roi_h, width, CV_8UC1);
    } else {
        component_mask.setTo(0);
    }
    for (int i = 0; i < count; ++i) {
        const int x = points[i].x;
        const int y = points[i].y - roi_y;
        if (0 <= x && x < width && 0 <= y && y < roi_h) {
            component_mask.at<uint8_t>(y, x) = 255;
        }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int num_labels =
        cv::connectedComponentsWithStats(component_mask, labels, stats, centroids, 8, CV_32S);

    // Area-only CC filter. Previously this looped (num_labels × W × H) to
    // collect each component's pixels for a cv::minAreaRect aspect/major
    // check — ~50 ms per frame on 540 × 960 with many small ground-
    // speckle components. The aspect filter also rejected curved /
    // T/L-shaped lines (the user explicitly noted this), so it served
    // neither speed nor accuracy. Now: O(num_labels) iterations only,
    // keep every component above kMinLineComponentPixels. Shape
    // classification is the CUDA kernel's job upstream.
    std::vector<uint8_t> keep_component(num_labels, 0);
    int kept_components = 0;
    for (int label = 1; label < num_labels; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < kMinLineComponentPixels) {
            continue;
        }
        keep_component[label] = 1;
        ++kept_components;
    }

    std::vector<int2> kept_points;
    kept_points.reserve(count);
    for (int i = 0; i < count; ++i) {
        const int x = points[i].x;
        const int y = points[i].y - roi_y;
        if (0 <= x && x < width && 0 <= y && y < roi_h) {
            const int label = labels.at<int>(y, x);
            if (label > 0 && keep_component[label]) {
                kept_points.push_back(points[i]);
            }
        }
    }

    delete[] filtered_points;
    *filtered_count = static_cast<int>(kept_points.size());
    filtered_points = new int2[std::max(1, *filtered_count)];
    for (int i = 0; i < *filtered_count; ++i) {
        filtered_points[i] = kept_points[i];
    }

    RCLCPP_DEBUG(
        rclcpp::get_logger("lines"),
        "Filtered line pixels from %d to %d using %d kept connected components",
        count, *filtered_count, kept_components);

    if (kept_component_count) {
        *kept_component_count = kept_components;
    }

    return std::make_pair(filtered_points, filtered_count);
}

}  // namespace

// Error function and macro borrowed from 
// https://github.com/jiekebo/CUDA-By-Example/blob/master/common/book.h

// thank you 

/**
 * Detects and returns pixels likely belonging to lines. 
 * Most noise detected is either far outside the course, or is part of an obstacle.
 * Thus, we are happy to map these pixels as obstacles on the map, since the robot will avoid them anyways. 
 * 
 */
std::pair<int2*, int*> lines::detect_line_pixels(const cv::Mat &image,
                                                  double brightness_threshold,
                                                  double roi_min_y_fraction,
                                                  int    half_window,
                                                  float  sigma_threshold,
                                                  float  mew_threshold,
                                                  bool   debug_image_write_enabled,
                                                  lines::LinePixelDetectionStats * stats) {
    if (stats) {
        *stats = lines::LinePixelDetectionStats();
    }

    // convert to grayscale
    cv::Mat gray_img;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4) {
        cv::cvtColor(image, gray_img, cv::COLOR_BGRA2GRAY);
    }
    else {
        gray_img = image;
    }
    
    int height = gray_img.rows;
    int width = gray_img.cols;
    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "input image r x c : %d x %d ", height, width);

    // Validate image dimensions
    if (height <= 0 || width <= 0 || height > 10000 || width > 10000) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Invalid image dimensions: %dx%d", height, width);
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Validate image data
    if (gray_img.data == nullptr || gray_img.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Empty image data");
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Validate continuous data
    if (!gray_img.isContinuous()) {
        RCLCPP_WARN(rclcpp::get_logger("lines"), "Image not continuous, making copy");
        gray_img = gray_img.clone();
    }

    const int roi_min_y = std::clamp(
        static_cast<int>(
            std::round(static_cast<double>(height) *
                       std::clamp(roi_min_y_fraction, 0.0, 1.0))),
        0,
        height);
    if (roi_min_y > 0) {
        // Crop before both the host-side mask and CUDA local-statistics
        // image, so the ignored rows cannot affect thresholding,
        // connected components, or windows straddling the ROI boundary.
        gray_img = gray_img.clone();
        gray_img.rowRange(0, roi_min_y).setTo(0);
    }

    // The brightness pre-mask is now applied on-GPU inside cerias_kernel
    // directly from the uploaded grayscale image (see __get_integral_image,
    // which uploads g_line_bufs.u8_input). No host-side cv::threshold or
    // mask upload is needed — that removed a full-image host threshold and
    // a W*H host->device copy per frame.

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Computing integral images");

    // Pre-allocate persistent CUDA buffers (one-time at startup; the
    // ensure() call is a no-op on subsequent frames as long as the
    // image dimensions are constant). All device pointers below come
    // from the persistent set — we never cudaMalloc/cudaFree per
    // frame anymore. Failure here means the GPU is in a bad state and
    // we can't proceed.
    if (!g_line_bufs.ensure(width, height)) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"),
                     "Failed to allocate persistent CUDA buffers");
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Get integral images. __get_integral_image now writes into the
    // persistent g_line_bufs.integral / integral_sq buffers (no
    // allocation), so the returned pointers are non-owning views
    // valid for the lifetime of g_line_bufs.
    try {
        __get_integral_image(gray_img);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Integral image failed: %s", e.what());
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    Npp32f * integral    = g_line_bufs.integral;
    Npp64f * integral_sq = g_line_bufs.integral_sq;

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Integral images computed, launching kernel");

    const uint8_t *device_gray = g_line_bufs.u8_input;  // uploaded by __get_integral_image
    int2    *output            = g_line_bufs.output;
    int     *counter           = g_line_bufs.counter;

    cudaError_t err;

    // Only the counter needs zeroing each frame. The kernel writes the
    // output array at atomically-assigned indices [0, counter) and the
    // host reads back only that many entries, so the W*H*sizeof(int2)
    // output buffer never needs a per-frame memset (~4 MB at 540x960).
    err = cudaMemset(counter, 0, sizeof(int));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"),
                     "cudaMemset failed for counter: %s",
                     cudaGetErrorString(err));
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Persistent buffers staged, launching kernel");

    // Launch kernel. Brightness gating happens on-GPU from device_gray.
    cerias_kernel(
        device_gray,
        integral, integral_sq,
        output, counter,
        width, height,
        half_window,
        static_cast<float>(brightness_threshold),
        sigma_threshold,
        mew_threshold
    );

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Kernel launch failed: %s",
                     cudaGetErrorString(err));
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Kernel execution failed: %s",
                     cudaGetErrorString(err));
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Kernel executed successfully");

    // Copy results back. counter and output are device pointers into
    // the persistent buffer set; only the host-side return arrays
    // ('counter_return', 'output_return') are heap-allocated for the
    // caller to own.
    int *counter_return = new int;
    err = cudaMemcpy(counter_return, counter, sizeof(int),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"),
                     "cudaMemcpy failed for counter: %s",
                     cudaGetErrorString(err));
        delete counter_return;
        counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Detected %d line pixels", *counter_return);
    if (stats) {
        stats->raw_pixels = *counter_return;
    }

    int2 *output_return = new int2[std::max(1, *counter_return)];

    if (*counter_return > 0) {
        err = cudaMemcpy(output_return, output,
                         *counter_return * sizeof(int2),
                         cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("lines"),
                         "cudaMemcpy failed for output: %s",
                         cudaGetErrorString(err));
            delete[] output_return;
            delete counter_return;
            counter_return = new int;
            *counter_return = 0;
            output_return = new int2[1];
            return std::make_pair(output_return, counter_return);
        }
    }

    int kept_components = 0;
    auto filtered_results = filter_line_components(
        output_return, *counter_return, width, height, roi_min_y, &kept_components);
    delete[] output_return;
    delete counter_return;
    output_return = filtered_results.first;
    counter_return = filtered_results.second;
    if (stats) {
        stats->filtered_pixels = *counter_return;
        stats->kept_components = kept_components;
    }

    if (debug_image_write_enabled) {
        const std::string out_dir = "line_debug";
        std::filesystem::create_directories(out_dir);

        cv::Mat raw_bgr;
        if (image.channels() == 3) {
            raw_bgr = image.clone();
        } else if (image.channels() == 4) {
            cv::cvtColor(image, raw_bgr, cv::COLOR_BGRA2BGR);
        } else {
            cv::cvtColor(image, raw_bgr, cv::COLOR_GRAY2BGR);
        }

        // The brightness mask is no longer materialized on the host in the
        // hot path; recompute it here only for the debug dump.
        cv::Mat mask;
        cv::threshold(gray_img, mask, brightness_threshold, 255, cv::THRESH_BINARY);
        cv::imwrite(out_dir + "/line_mask.png", mask);

        cv::Mat lines_overlay = raw_bgr.clone();
        const int n = *counter_return;

        for (int i = 0; i < n; i++) {
            int x = output_return[i].x;
            int y = output_return[i].y;
            if (0 <= x && x < width && 0 <= y && y < height) {
                lines_overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                if (x + 1 < width) lines_overlay.at<cv::Vec3b>(y, x + 1) = cv::Vec3b(0, 0, 255);
                if (x - 1 >= 0)   lines_overlay.at<cv::Vec3b>(y, x - 1) = cv::Vec3b(0, 0, 255);
                if (y + 1 < height) lines_overlay.at<cv::Vec3b>(y + 1, x) = cv::Vec3b(0, 0, 255);
                if (y - 1 >= 0)     lines_overlay.at<cv::Vec3b>(y - 1, x) = cv::Vec3b(0, 0, 255);
            }
        }

        cv::imwrite(out_dir + "/line_raw.png", raw_bgr);
        cv::imwrite(out_dir + "/line_lines.png", lines_overlay);

        RCLCPP_INFO(
            rclcpp::get_logger("lines"),
            "Updated debug images in %s (line points=%d)",
            out_dir.c_str(), n);
    }

    // Device buffers are persistent (see g_line_bufs) — no cleanup
    // here. Host-side return arrays are owned by the caller.
    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Done, returning results");

    return std::make_pair(output_return, counter_return);
}

/**
 * Retrieves the integral image and square integral image from a grayscale cv2 image
 * 
 * RETURNS DEVICE POINTERS.... this function is meant to used internally and keep the data on device for downstream processing
 * do NOT call this function and then dereference anything without memcpy-ing back to host.
 * 
 */
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img) {

    int width = gray_img.cols;
    int height = gray_img.rows;

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Computing integral image for %dx%d image", width, height);

    // Validate dimensions
    if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
        throw std::runtime_error("Invalid image dimensions for integral image");
    }

    // CAREFUL, WE ARE CASTING TO 8 BIT PIXELS HERE, MAKE SURE INPUT IS 8 BIT
    if (gray_img.type() != CV_8UC1) {
        throw std::runtime_error("Input image must be CV_8UC1");
    }

    // Caller (detect_line_pixels) is responsible for calling
    // g_line_bufs.ensure(width, height) before this function. The
    // input/output device buffers below are taken from that persistent
    // set — no cudaMalloc per frame.
    Npp8u   *device_input_img = g_line_bufs.u8_input;
    Npp32f  *result           = g_line_bufs.integral;
    Npp64f  *result_sq        = g_line_bufs.integral_sq;
    if (device_input_img == nullptr || result == nullptr || result_sq == nullptr) {
        throw std::runtime_error(
            "g_line_bufs not initialized; caller must ensure() before __get_integral_image()");
    }

    const size_t image_size = static_cast<size_t>(width) * height * sizeof(Npp8u);
    cudaError_t err = cudaMemcpy(device_input_img, gray_img.ptr<Npp8u>(),
                                 image_size, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(err));
    }

    // The integral/integral_sq buffers are zeroed once in
    // LineDeviceBuffers::ensure(); nppiSqrIntegral below rewrites the full
    // interior and the zero border is invariant, so no per-frame memset.

    // set nsrcstep, ndststep, and roi
    size_t nsrcstep = width * sizeof(Npp8u);
    size_t ndststep = (width + 1) * sizeof(Npp32f);
    size_t nsqrstep = (width + 1) * sizeof(Npp64f);
    NppiSize roi = { width, height };

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Calling nppiSqrIntegral_8u32f64f_C1R");

    NppStatus status = nppiSqrIntegral_8u32f64f_C1R(
        device_input_img,
        nsrcstep,
        result,
        ndststep,
        result_sq,
        nsqrstep,
        roi,
        0,
        0
    );

    if (status != NPP_SUCCESS) {
        std::string error_msg = "nppiSqrIntegral failed with code: " + std::to_string(status);
        throw std::runtime_error(error_msg);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Integral image computed successfully");

    return std::make_pair(result, result_sq);
}
