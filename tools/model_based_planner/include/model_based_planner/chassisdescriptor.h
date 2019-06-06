#ifndef CHASSISDESCRIPTOR_H
#define CHASSISDESCRIPTOR_H

#include "cv_aligned_mat.h"


/**
 * @brief ChassisDescriptor for one vehicle orientation
 */
struct ChassisDescriptor
{

    /**
     * @brief location of the chassis image
     */
    cv::Point2f centerImg_;

    /**
     * @brief x and y direction on the chassis image (the vehicle is rotated, the image still has normal top right coordinates)
     */
    cv::Point2f dirX_, dirY_;

    /**
     * @brief Chassis image
     */
    CVAlignedMat::ptr image_;

};

#endif // CHASSISDESCRIPTOR_H
