#include "autonav_detection/detection.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{

constexpr int kMinLineComponentPixels = 20;
constexpr float kMinLineMajorAxisPx = 12.0F;
constexpr float kMinLineAspectRatio = 1.8F;
constexpr float kMaxCompactFillRatio = 0.48F;

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
    float   *float_input = nullptr;
    uint8_t *mask        = nullptr;
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
    if (cudaMalloc(reinterpret_cast<void**>(&float_input),
                   total * sizeof(float)) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&mask),
                   total) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&output),
                   total * sizeof(int2)) != cudaSuccess) { freeAll(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&counter),
                   sizeof(int)) != cudaSuccess) { freeAll(); return false; }

    width = new_w;
    height = new_h;
    return true;
}

void LineDeviceBuffers::freeAll()
{
    if (u8_input)    { cudaFree(u8_input);    u8_input    = nullptr; }
    if (integral)    { cudaFree(integral);    integral    = nullptr; }
    if (integral_sq) { cudaFree(integral_sq); integral_sq = nullptr; }
    if (float_input) { cudaFree(float_input); float_input = nullptr; }
    if (mask)        { cudaFree(mask);        mask        = nullptr; }
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
    int min_component_pixels,
    int * kept_component_count)
{
    int *filtered_count = new int(0);
    int2 *filtered_points = new int2[1];

    if (!points || count <= 0 || width <= 0 || height <= 0) {
        return std::make_pair(filtered_points, filtered_count);
    }

    // Persist the component_mask between frames; only allocate on size
    // change. cv::Mat::zeros allocates fresh memory every frame; setTo(0)
    // reuses the buffer. At 540x960 this is ~500 KB saved + a memset.
    static cv::Mat component_mask;
    if (component_mask.rows != height || component_mask.cols != width ||
        component_mask.type() != CV_8UC1) {
        component_mask = cv::Mat::zeros(height, width, CV_8UC1);
    } else {
        component_mask.setTo(0);
    }
    for (int i = 0; i < count; ++i) {
        const int x = points[i].x;
        const int y = points[i].y;
        if (0 <= x && x < width && 0 <= y && y < height) {
            component_mask.at<uint8_t>(y, x) = 255;
        }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int num_labels =
        cv::connectedComponentsWithStats(component_mask, labels, stats, centroids, 8, CV_32S);

    // O(num_labels) component filter. Keep elongated components and sparse
    // components (curves / T / L junctions), reject compact filled blobs.
    // A plain aspect-ratio gate rejects junctions; area-only admits glare.
    std::vector<uint8_t> keep_component(num_labels, 0);
    int kept_components = 0;
    for (int label = 1; label < num_labels; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < std::max(kMinLineComponentPixels, min_component_pixels)) {
            continue;
        }
        const int bbox_w = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int bbox_h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const int major_axis = std::max(bbox_w, bbox_h);
        const int minor_axis = std::max(1, std::min(bbox_w, bbox_h));
        const float aspect =
            static_cast<float>(major_axis) / static_cast<float>(minor_axis);
        const float fill_ratio =
            static_cast<float>(area) /
            static_cast<float>(std::max(1, bbox_w * bbox_h));
        const bool line_like =
            major_axis >= kMinLineMajorAxisPx &&
            (aspect >= kMinLineAspectRatio || fill_ratio <= kMaxCompactFillRatio);
        if (!line_like) {
            continue;
        }
        keep_component[label] = 1;
        ++kept_components;
    }

    std::vector<int2> kept_points;
    kept_points.reserve(count);
    for (int i = 0; i < count; ++i) {
        const int x = points[i].x;
        const int y = points[i].y;
        if (0 <= x && x < width && 0 <= y && y < height) {
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

cv::Mat lines::build_line_candidate_mask(
    const cv::Mat & image,
    double brightness_threshold,
    const lines::LineColorMaskConfig & config,
    bool include_brightness_mask)
{
    cv::Mat gray_img;
    cv::Mat bgr_img;
    if (image.channels() == 3) {
        bgr_img = image;
        cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, bgr_img, cv::COLOR_BGRA2BGR);
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = image;
    }

    cv::Mat candidate_mask = cv::Mat::zeros(gray_img.rows, gray_img.cols, CV_8UC1);
    if (include_brightness_mask) {
        cv::threshold(gray_img, candidate_mask, brightness_threshold, 255, cv::THRESH_BINARY);
    }

    if (config.enable_color_mask && !bgr_img.empty()) {
        cv::Mat hsv;
        cv::cvtColor(bgr_img, hsv, cv::COLOR_BGR2HSV);

        if (config.detect_white) {
            cv::Mat white_mask;
            cv::inRange(
                hsv,
                cv::Scalar(0, 0, std::clamp(config.white_value_min, 0, 255)),
                cv::Scalar(179, std::clamp(config.white_saturation_max, 0, 255), 255),
                white_mask);
            cv::bitwise_or(candidate_mask, white_mask, candidate_mask);
        }

        if (config.detect_yellow) {
            cv::Mat yellow_mask;
            const int hue_min = std::clamp(config.yellow_hue_min, 0, 179);
            const int hue_max = std::clamp(config.yellow_hue_max, 0, 179);
            const int sat_min = std::clamp(config.yellow_saturation_min, 0, 255);
            const int val_min = std::clamp(config.yellow_value_min, 0, 255);
            if (hue_min <= hue_max) {
                cv::inRange(
                    hsv,
                    cv::Scalar(hue_min, sat_min, val_min),
                    cv::Scalar(hue_max, 255, 255),
                    yellow_mask);
            } else {
                cv::Mat low_mask;
                cv::Mat high_mask;
                cv::inRange(
                    hsv,
                    cv::Scalar(0, sat_min, val_min),
                    cv::Scalar(hue_max, 255, 255),
                    low_mask);
                cv::inRange(
                    hsv,
                    cv::Scalar(hue_min, sat_min, val_min),
                    cv::Scalar(179, 255, 255),
                    high_mask);
                cv::bitwise_or(low_mask, high_mask, yellow_mask);
            }
            cv::bitwise_or(candidate_mask, yellow_mask, candidate_mask);
        }
    }

    const int close_size = std::max(0, config.morph_close_size);
    if (close_size > 1) {
        const int k = close_size | 1;
        const cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        cv::morphologyEx(candidate_mask, candidate_mask, cv::MORPH_CLOSE, kernel);
    }
    const int open_size = std::max(0, config.morph_open_size);
    if (open_size > 1) {
        const int k = open_size | 1;
        const cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        cv::morphologyEx(candidate_mask, candidate_mask, cv::MORPH_OPEN, kernel);
    }

    return candidate_mask;
}

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
                                                  int    half_window,
                                                  float  sigma_threshold,
                                                  float  mew_threshold,
                                                  bool   debug_image_write_enabled,
                                                  lines::LinePixelDetectionStats * stats,
                                                  const lines::LineColorMaskConfig & color_config) {
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

    cv::Mat mask;
    cv::threshold(gray_img, mask, brightness_threshold, 255, cv::THRESH_BINARY);

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Threshold complete, computing integral images");

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

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Integral images computed, uploading float input + mask");

    // Convert grayscale to float on host, then upload.
    cv::Mat gray_float;
    gray_img.convertTo(gray_float, CV_32F);
    if (gray_float.empty() || gray_float.data == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"), "Float conversion failed");
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    float   *input_image_device = g_line_bufs.float_input;
    uint8_t *device_mask        = g_line_bufs.mask;
    int2    *output             = g_line_bufs.output;
    int     *counter            = g_line_bufs.counter;
    const size_t total          = static_cast<size_t>(width) * height;

    cudaError_t err;

    err = cudaMemcpy(input_image_device, gray_float.ptr<float>(),
                     total * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"),
                     "cudaMemcpy failed for input image: %s",
                     cudaGetErrorString(err));
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    err = cudaMemcpy(device_mask, mask.ptr<uint8_t>(),
                     total, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"),
                     "cudaMemcpy failed for mask: %s",
                     cudaGetErrorString(err));
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

    // Output buffer and counter must be zeroed every frame (they
    // accumulate during the kernel).
    err = cudaMemset(output, 0, total * sizeof(int2));
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("lines"),
                     "cudaMemset failed for output: %s",
                     cudaGetErrorString(err));
        int *counter_return = new int;
        *counter_return = 0;
        int2 *output_return = new int2[1];
        return std::make_pair(output_return, counter_return);
    }

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

    // Launch kernel
    cerias_kernel(
        input_image_device,
        integral, integral_sq,
        device_mask,
        output, counter,
        width, height,
        half_window,
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

    RCLCPP_DEBUG(rclcpp::get_logger("lines"), "Detected %d CERIAS line pixels", *counter_return);

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

    cv::Mat combined_mask = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < *counter_return; ++i) {
        const int x = output_return[i].x;
        const int y = output_return[i].y;
        if (0 <= x && x < width && 0 <= y && y < height) {
            combined_mask.at<uint8_t>(y, x) = 255;
        }
    }
    cv::Mat color_mask = lines::build_line_candidate_mask(
        image, brightness_threshold, color_config, false);
    cv::bitwise_or(combined_mask, color_mask, combined_mask);

    std::vector<cv::Point> mask_points;
    cv::findNonZero(combined_mask, mask_points);
    int *combined_count = new int(static_cast<int>(mask_points.size()));
    int2 *combined_points = new int2[std::max(1, *combined_count)];
    for (int i = 0; i < *combined_count; ++i) {
        combined_points[i].x = mask_points[i].x;
        combined_points[i].y = mask_points[i].y;
    }
    if (stats) {
        stats->raw_pixels = *combined_count;
    }

    int kept_components = 0;
    auto filtered_results = filter_line_components(
        combined_points, *combined_count, width, height,
        color_config.min_component_pixels, &kept_components);
    delete[] output_return;
    delete counter_return;
    delete[] combined_points;
    delete combined_count;
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

    const size_t result_size = static_cast<size_t>(height + 1) * (width + 1) * sizeof(Npp32f);
    err = cudaMemset(result, 0, result_size);
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("cudaMemset failed (integral): ") + cudaGetErrorString(err));
    }

    const size_t result_sq_size = static_cast<size_t>(height + 1) * (width + 1) * sizeof(Npp64f);
    err = cudaMemset(result_sq, 0, result_sq_size);
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("cudaMemset failed (integral_sq): ") + cudaGetErrorString(err));
    }

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
