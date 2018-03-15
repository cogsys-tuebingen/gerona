#ifndef WHEELDESCRIPTOR_H
#define WHEELDESCRIPTOR_H

#include "cv_aligned_mat.h"


/**
 * @brief Wheel descriptor for one wheel orientation
 */
struct WheelDescriptor
{

    cv::Point2f centerImg_,jointPosImg_;

    cv::Point2f dirX_, dirY_;

    int numImagePixels_;
    float numImagePixelsInv_;

    CVAlignedMat::ptr image_;
};

#endif // WHEELDESCRIPTOR_H
