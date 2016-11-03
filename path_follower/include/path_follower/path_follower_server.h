#ifndef PATH_FOLLOWER_SERVER_H
#define PATH_FOLLOWER_SERVER_H

/// SYSTEM
#include <actionlib/server/simple_action_server.h>
#include <path_msgs/FollowPathAction.h>

class PathFollower;

class PathFollowerServer
{
public:
    PathFollowerServer(PathFollower& follower);

    void spin();
    void update();

    //! Callback for new follow_path action goals.
    void followPathGoalCB();
    //! Callback for follow_path action preemption.
    void followPathPreemptCB();

private:
    PathFollower& follower_;

    typedef actionlib::SimpleActionServer<path_msgs::FollowPathAction> FollowPathServer;

    //! Action server that communicates with path_control (or who ever sends actions)
    FollowPathServer follow_path_server_;

    path_msgs::FollowPathGoalConstPtr latest_goal_;
};

#endif // PATH_FOLLOWER_SERVER_H
