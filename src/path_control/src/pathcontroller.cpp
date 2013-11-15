#include "pathcontroller.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

PathController::PathController(ros::NodeHandle &nh):
    node_handle_(nh),
    navigate_to_goal_server_(nh, "navigate_to_goal", false),
    follow_path_client_("follow_path", false)
{
    //TODO: GoalCallback vs ExecuteCallback?
    //navigate_to_goal_server_.registerGoalCallback(boost::bind(&PathController::NavToGoalActionCallback, this, _1));
    navigate_to_goal_server_.start();

    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/goal", 0);
}

void PathController::NavToGoalActionCallback(const path_msgs::NavigateToGoalActionGoalConstPtr &goal)
{
    // ...

    goal_pub_.publish(goal->goal.goal_pose);
    //TODO: how to catch the path published by path_planner and ensuring, that the received path is really the one
    // associated with the current goal? Maybe by using the stamp?


    // assume we already have the path
    nav_msgs::Path path;

    path_msgs::FollowPathActionGoal path_action_goal;
    path_action_goal.goal.path = path;
}
