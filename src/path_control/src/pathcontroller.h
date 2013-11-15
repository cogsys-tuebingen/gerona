#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <ros/ros.h>

class PathController
{
public:
    PathController(ros::NodeHandle &nh);

private:
    ros::NodeHandle node_handle_;

    //! Publishes goal as PoseStamped for path_planner and rviz.
    ros::Publisher goal_pub_;
};

#endif // PATHCONTROLLER_H
