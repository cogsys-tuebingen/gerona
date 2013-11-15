#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/FollowPathAction.h>

class PathController
{


public:
    PathController(ros::NodeHandle &nh);

private:
    typedef actionlib::SimpleActionServer<path_msgs::NavigateToGoalAction> NavToGoalServer;
    typedef actionlib::SimpleActionClient<path_msgs::FollowPathAction> FollowPathClient;

    ros::NodeHandle node_handle_;

    //! Action server that communicates with the high level control (the node that sends the goal points).
    NavToGoalServer navigate_to_goal_server_;

    //! Action client to communicate with the path_follower package.
    FollowPathClient follow_path_client_;

    //! Publishes goal as PoseStamped for path_planner and rviz.
    ros::Publisher goal_pub_;

    void NavToGoalActionCallback(const path_msgs::NavigateToGoalActionGoalConstPtr &goal);

};

#endif // PATHCONTROLLER_H
