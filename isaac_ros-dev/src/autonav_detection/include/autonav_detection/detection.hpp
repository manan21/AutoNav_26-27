
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

// Tunables: brightness_threshold gates the pre-mask; roi_min_y_fraction
// zeros upper image rows before CUDA; half_window / sigma_threshold /
// mew_threshold drive the CERIAS kernel. Defaults are the historical
// compile-time values (220 / 0.0 / 3 / 5.0 / 200.0). All live in
// line_detector.yaml and are wired through node.cpp.
std::pair<int2*, int*> detect_line_pixels(const cv::Mat & image,
                                          double brightness_threshold = 220.0,
                                          double roi_min_y_fraction = 0.0,
                                          int    half_window = 3,
                                          float  sigma_threshold = 5.0f,
                                          float  mew_threshold = 200.0f,
                                          bool   debug_image_write_enabled = false,
                                          LinePixelDetectionStats * stats = nullptr);
}
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img);



#endif
