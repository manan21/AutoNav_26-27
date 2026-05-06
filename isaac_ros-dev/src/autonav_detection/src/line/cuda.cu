/**
 * Cuda kernels for fast Line detection processing
 *
 * Tunable knobs (half_window, sigma_threshold, mew_threshold) are passed
 * in from the host as kernel arguments — they live in line_detector.yaml
 * and are read by node.cpp via declare_parameter.
 */
#include "autonav_detection/cuda.cuh"


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
