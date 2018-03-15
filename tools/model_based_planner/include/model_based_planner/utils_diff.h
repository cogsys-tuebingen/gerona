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

#ifdef USE_AVX2
#include "utils_diff_axv.h"
#else
#include "utils_diff_sse.h"
#endif







#endif // UTILS_DIFF_H
