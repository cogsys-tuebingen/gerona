#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/FollowPathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

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

    //! Subscibes for the paths published by path_planner
    ros::Subscriber path_sub_;

    /**
     * @brief Timestamp of the last goal that was send to path_planner
     *
     * Since the communication between path_control and path_planner is done via simple messages instead of an service
     * (which is done because goals and paths can then easily be displayed in rviz), there is no nonambiguous
     * association between the goal, sent to the planner, and the resulting path.
     * To solve this, the planner sets the timestamp of the path to the one of the goal, so it can be used as a mostly
     * unique id to associate path with goals. Thus the timestamp of the goal is saved here, so it can be compared with
     * the timestamps of incomming paths.
     *
     * Set this member to Time(0), while there is no outstanding path.
     */
    ros::Time goal_timestamp_;

    nav_msgs::PathConstPtr requested_path_;

    void navToGoalActionCallback(const path_msgs::NavigateToGoalActionGoalConstPtr &goal);

    //! Callback for the paths published e.g. by path_planner
    void pathCallback(const nav_msgs::PathConstPtr &path);

    //! Send a goal pose to path_follower and wait for the resulting path.
    /** \todo Timeout! */
    void waitForPath(const geometry_msgs::PoseStamped &goal_pose);
};

#endif // PATHCONTROLLER_H
