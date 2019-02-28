#ifndef ROBOTDESCRIPTORDT_H
#define ROBOTDESCRIPTORDT_H

#include <opencv2/core/core.hpp>

/**
 * @brief Robot descriptor for one vehicle orientation
 */
struct RobotDescriptorDT
{

    inline cv::Point2f GetPixelPos(const int &idx){ return testPositionsImage_[idx]; }

    std::vector<cv::Point2f> testPositionsImage_;

    cv::Point2f baseLinkPosImage_;

    cv::Mat rotMat_;

    float sina_,cosa_;
};

#endif // ROBOTDESCRIPTOR_H
