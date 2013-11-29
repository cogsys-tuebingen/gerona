#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <path_msgs/FollowPathAction.h>

class PathFollower
{
public:
    PathFollower(ros::NodeHandle &nh);

private:
    typedef actionlib::SimpleActionServer<path_msgs::FollowPathAction> FollowPathServer;

    ros::NodeHandle node_handle_;

    //! Action server that communicates with path_control (or who ever sends actions)
    FollowPathServer follow_path_server_;


    void executeFollowPathCB(const path_msgs::FollowPathGoalConstPtr &goal);
};

#endif // PATHFOLLOWER_H
