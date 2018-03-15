#ifndef CHASSISDESCRIPTOR_H
#define CHASSISDESCRIPTOR_H

#include "cv_aligned_mat.h"


/**
 * @brief ChassisDescriptor for one vehicle orientation
 */
struct ChassisDescriptor
{

    cv::Point2f centerImg_;

    cv::Point2f dirX_, dirY_;

    CVAlignedMat::ptr image_;

};

#endif // CHASSISDESCRIPTOR_H
