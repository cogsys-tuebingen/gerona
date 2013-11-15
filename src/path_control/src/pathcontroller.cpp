#include "pathcontroller.h"

PathController::PathController(ros::NodeHandle &nh):
    node_handle_(nh),
    navigate_to_goal_server_(nh, "navigate_to_goal", false),
    follow_path_client_("follow_path", false),
    goal_timestamp_(ros::Time(0))
{
    //TODO: GoalCallback vs ExecuteCallback?
    //navigate_to_goal_server_.registerGoalCallback(boost::bind(&PathController::NavToGoalActionCallback, this, _1));
    navigate_to_goal_server_.start();

    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/goal", 0);
    path_sub_ = nh.subscribe<nav_msgs::Path>("/path", 10, &PathController::pathCallback, this);
}

void PathController::navToGoalActionCallback(const path_msgs::NavigateToGoalActionGoalConstPtr &goal)
{
    // ...

    waitForPath(goal->goal.goal_pose);

    path_msgs::FollowPathActionGoal path_action_goal;
    path_action_goal.goal.path = *requested_path_;
    // ... set some more parameters here...

    //follow_path_client_.sendGoal(path_action_goal, /* ... */);

}

void PathController::pathCallback(const nav_msgs::PathConstPtr &path)
{
    //TODO: how to handle unexpected paths? They should be executed but maybe only if the path_follower is idle?
    // Maybe extra mode for this?

    if (goal_timestamp_.isZero()) {
        //TODO: what to do here?
    } else {
        if (path->header.stamp == goal_timestamp_) {
            requested_path_ = path;
            // reset to 0 to signalise, that there is no outstanding path
            goal_timestamp_ = ros::Time(0);
        }
        // else: drop this path (= do nothing)
    }
}

void PathController::waitForPath(const geometry_msgs::PoseStamped &goal_pose)
{
    //TODO: Can there be concurrency problems? I think not, but better think a bit more deeply about it.

    goal_timestamp_ = goal_pose.header.stamp;
    goal_pub_.publish(goal_pose);

    while (!goal_timestamp_.isZero());
}
