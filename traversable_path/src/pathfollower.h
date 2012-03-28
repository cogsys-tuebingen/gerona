#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include "traversable_path/LaserScanClassification.h"

class PathFollower
{
public:
    PathFollower();

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber subscribe_scan_classification_;

    void scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan_classification);
};

#endif // PATHFOLLOWER_H
