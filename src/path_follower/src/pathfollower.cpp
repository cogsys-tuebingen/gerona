#include "pathfollower.h"

PathFollower::PathFollower(ros::NodeHandle &nh):
    node_handle_(nh),
    follow_path_server_(nh, "follow_path", boost::bind(&PathFollower::executeFollowPathCB, this, _1), false)
{
    follow_path_server_.start();

    ROS_INFO("Initialisation done.");
}

void PathFollower::executeFollowPathCB(const path_msgs::FollowPathGoalConstPtr &goal)
{
    ROS_INFO("Start Action!!");

    // dummy feedback

    path_msgs::FollowPathFeedback feed;
    ros::Duration(1).sleep();
    follow_path_server_.publishFeedback(feed);
    ros::Duration(1).sleep();
    follow_path_server_.publishFeedback(feed);
    ros::Duration(3).sleep();

    path_msgs::FollowPathResult res;
    res.status = res.STATUS_SUCCESS;
    follow_path_server_.setSucceeded(res);
}
