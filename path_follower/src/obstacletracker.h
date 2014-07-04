#ifndef OBSTACLETRACKER_H
#define OBSTACLETRACKER_H

/// STL
#include <vector>

/// THIRD PARTY
#include <opencv2/imgproc/imgproc.hpp>

/// ROS
#include <ros/ros.h>

/// PROJECT
#include "maptransformer.h"

class ObstacleTracker
{
public:
    struct TrackedObstacle {
        std::vector<cv::Point> contour;
        cv::Point2f last_position;
        ros::Time time_of_first_sight;
    };

    ObstacleTracker():
        max_dist_(0.3f)
    {}

    void setMaxDist(float md)
    {
        max_dist_ = md;
    }

    std::vector<TrackedObstacle> getTrackedObstacles() const
    {
        return obstacles_;
    }


    void update(std::vector<cv::Point2f> obstacles);

private:
    float max_dist_;

    std::vector<TrackedObstacle> obstacles_;
};

#endif // OBSTACLETRACKER_H
