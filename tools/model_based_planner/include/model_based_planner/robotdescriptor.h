#ifndef ROBOTDESCRIPTOR_H
#define ROBOTDESCRIPTOR_H

#include <opencv2/core/core.hpp>

/**
 * @brief Robot descriptor for one vehicle orientation
 */
struct RobotDescriptor
{

    inline cv::Point2f GetPixelPos(const int &idx){ return wheelPositionsImage_[idx]; }


    std::vector<cv::Point2f> wheelPositionsImage_;

    cv::Point2f chassisPosImage_;


    cv::Point2f baseLinkPosImage_;


    cv::Mat rotMat_;

    float sina_,cosa_;
};

#endif // ROBOTDESCRIPTOR_H
