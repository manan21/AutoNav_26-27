
#ifndef CUDA_CUH
#define CUDA_CUH

#include <cuda_runtime.h>
#include <npp.h>
#include <cstdint>
#include <cstdio>


// CERIAS line-pixel kernel.
//
// Per-pixel local statistics inside a (2 * half_window + 1)^2 window
// computed from the integral images. A pixel passes the test (and gets
// emitted as a line pixel) iff the local stddev < sigma_threshold AND
// local mean > mew_threshold. Defaults are wired through node.cpp from
// the line_detector.yaml config; they are not compile-time constants.
__global__ void __cerias_kernel(float * gray_img,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * mask,
                             int2 * output,
                             int * counter,
                             int width, int height,
                             int half_window,
                             float sigma_threshold,
                             float mew_threshold);


extern "C" void cerias_kernel(float * gray_img,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * mask,
                             int2 * output,
                             int * counter,
                             int width, int height,
                             int half_window,
                             float sigma_threshold,
                             float mew_threshold);

#endif
