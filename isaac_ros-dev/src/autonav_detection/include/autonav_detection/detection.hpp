
#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "cuda.cuh"
#include <utility>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

[[maybe_unused]] static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("line_logging"), "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
                         };


#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

namespace lines {
struct LinePixelDetectionStats {
    int raw_pixels = 0;
    int filtered_pixels = 0;
    int kept_components = 0;
};

struct LineColorMaskConfig {
    bool enable_color_mask = true;
    bool detect_white = true;
    bool detect_yellow = true;
    int white_value_min = 165;
    int white_saturation_max = 110;
    int yellow_hue_min = 12;
    int yellow_hue_max = 45;
    int yellow_saturation_min = 50;
    int yellow_value_min = 90;
    int morph_close_size = 5;
    int morph_open_size = 0;
    int min_component_pixels = 20;
};

cv::Mat build_line_candidate_mask(const cv::Mat & image,
                                  double brightness_threshold,
                                  const LineColorMaskConfig & config = LineColorMaskConfig(),
                                  bool include_brightness_mask = true);

// Tunables: brightness_threshold gates the pre-mask; half_window /
// sigma_threshold / mew_threshold drive the CERIAS kernel. Defaults are
// the historical compile-time values (220 / 3 / 5.0 / 200.0). All four
// live in line_detector.yaml and are wired through node.cpp.
std::pair<int2*, int*> detect_line_pixels(const cv::Mat & image,
                                          double brightness_threshold = 220.0,
                                          int    half_window = 3,
                                          float  sigma_threshold = 5.0f,
                                          float  mew_threshold = 200.0f,
                                          bool   debug_image_write_enabled = false,
                                          LinePixelDetectionStats * stats = nullptr,
                                          const LineColorMaskConfig & color_config = LineColorMaskConfig());
}
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img);



#endif
