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
    ROS_INFO("Start Action!! [%d]", goal->debug_test);

    // dummy feedback

    path_msgs::FollowPathFeedback feed;
    feed.debug_test = goal->debug_test;

    for (int i = 0; i<4; ++i) {
        ros::Duration(1).sleep();

        if (follow_path_server_.isPreemptRequested()) {
            ROS_INFO("Preemt goal [%d].\n---------------------", goal->debug_test);
            path_msgs::FollowPathResult res;
            res.status = res.STATUS_ABORTED;
            res.debug_test = goal->debug_test;
            follow_path_server_.setPreempted(res);
            return;
        }

        ROS_INFO("Feedback [%d]", goal->debug_test);
        follow_path_server_.publishFeedback(feed);
    }

    ros::Duration(3).sleep();

    path_msgs::FollowPathResult res;
    res.status = res.STATUS_SUCCESS;
    res.debug_test = goal->debug_test;
    ROS_INFO("Finished [%d].\n---------------------", goal->debug_test);
    follow_path_server_.setSucceeded(res);
}
