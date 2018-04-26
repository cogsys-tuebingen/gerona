#ifndef UTILS_POSE_ESTIMATOR_H
#define UTILS_POSE_ESTIMATOR_H

// THIRD PARTY
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <opencv2/core/core.hpp>


// Project
#include "poseestimator.h"


class UtilsPoseEstimator
{
public:
    UtilsPoseEstimator();

    void Initialize(ros::NodeHandle &nodeP);

    bool UseEstimate() {return usePoseEstimator_;}

    void GetEstimate(tf::StampedTransform &robotPose);

    void UpdateLocalMap(const cv::Mat &dem, const cv::Point2f &mapOrigin);

    bool GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans);

private:

    bool usePoseEstimator_;
    bool hasPoseEstimator_;
    PoseEstimator poseEstimator_;

    tf::TransformListener tf_listener;
    std::string baseFrame_;
    std::string mapFrame_;
    std::string localMapFrame_;
    cv::Point2f curMapPos_;

    int mapCounter_;
    int mapSkipVal_;

};






#endif //UTILS_POSE_ESTIMATOR_H
