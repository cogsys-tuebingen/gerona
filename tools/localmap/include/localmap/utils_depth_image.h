#ifndef UTILS_DEPTH_IMAGE_H
#define UTILS_DEPTH_IMAGE_H


#include <mm_malloc.h>
#include <memory>
#include <opencv2/core/core.hpp>

/**
 * @brief Helper class for cropping the depth image
 */
class UtilsDepthImage
{
public:
    /**
     * @brief Some structured light sensors produce invalid measurements in the first few columns, setting these to zero
     */
    static void RemoveLeftBorderNoise2(cv::Mat image, int numCols, int startRow)
    {
        float *imgPtr;
        const int size = numCols*sizeof(float);

        for (int yl = startRow; yl < image.rows;++yl)
        {
            imgPtr = image.ptr<float>(yl);
            memset(imgPtr,0,size);
        }

    }
    /**
     * @brief Some structured light sensors produce invalid measurements in the first few columns, setting these to NAN
     */
    static void RemoveLeftBorderNoise(cv::Mat &image, int numCols, int startRow)
    {
        float *imgPtr;
        int xl;
        for (int yl = startRow; yl < image.rows;++yl)
        {
            imgPtr = image.ptr<float>(yl);
            for (xl = 0; xl < numCols;++xl) imgPtr[xl] =NAN;
        }

    }

    /**
     * @brief Crop the depth image to given rectangle
     */
    static void RemoveWindow(cv::Mat &image, int startCol, int endCol, int startRow, int endRow)
    {
        float *imgPtr;
        int xl;
        for (int yl = startRow; yl < endRow;++yl)
        {
            imgPtr = image.ptr<float>(yl);
            for (xl = startCol; xl < endCol;++xl) imgPtr[xl] =NAN;
        }

    }


    static inline void SetToZero(cv::Mat &mat)
    {
        memset(mat.data,0,mat.step*mat.rows);

    }
};


#endif //UTILS_DEPTH_IMAGE_H
