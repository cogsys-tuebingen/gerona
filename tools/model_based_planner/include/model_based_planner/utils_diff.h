#ifndef UTILS_DIFF_H
#define UTILS_DIFF_H


#include <mm_malloc.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * @brief Include SIMD functions depending on requested CPU architecture
 */


#if defined(USE_AVX2) && defined(__AVX2__)
#include "utils_diff_axv.h"
#elif __SSE4_2__
#include "utils_diff_sse.h"
#else
#include "utils_diff_nosimd.h"
#endif








#endif // UTILS_DIFF_H
