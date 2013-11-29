#include "pathcontroller.h"

PathController::PathController(ros::NodeHandle &nh):
    node_handle_(nh),
    navigate_to_goal_server_(nh, "navigate_to_goal", boost::bind(&PathController::navToGoalActionCallback, this, _1), false),
    follow_path_client_("follow_path"),
    goal_timestamp_(ros::Time(0))
{
    follow_path_client_.waitForServer();

    //TODO: GoalCallback vs ExecuteCallback?
    //navigate_to_goal_server_.registerGoalCallback(boost::bind(&PathController::navToGoalActionCallback, this, _1));
    navigate_to_goal_server_.start();

    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/goal", 0);
    path_sub_ = nh.subscribe<nav_msgs::Path>("/path", 10, &PathController::pathCallback, this);

    ROS_INFO("Initialisation done.");
}

void PathController::navToGoalActionCallback(const path_msgs::NavigateToGoalGoalConstPtr &goal)
{
    ROS_INFO("Start Action!!");
    // ...

    // TODO: handle preempt request!

    //FIXME: uncomment after testing!
    //waitForPath(goal->goal_pose);

    path_msgs::FollowPathGoal path_action_goal;
    //path_action_goal.path = *requested_path_;
    // ... set some more parameters here...

    follow_path_client_.sendGoal(path_action_goal,
                                 boost::bind(&PathController::followPathDoneCB, this, _1, _2),
                                 boost::bind(&PathController::followPathActiveCB, this),
                                 boost::bind(&PathController::followPathFeedbackCB, this, _1));
    // block this function, until the action is running
    follow_path_client_.waitForResult(); //TODO: Timeout?
}

void PathController::pathCallback(const nav_msgs::PathConstPtr &path)
{
    //TODO: how to handle unexpected paths? They should be executed but maybe only if the path_follower is idle?
    // Maybe extra mode for this?

    if (goal_timestamp_.isZero()) { // unexpected path -> case 2
        //TODO: what to do here?
        //if (action is running) return;
        //else executePath(); <- nein, das blockiert den callback...
    } else { // expected path -> case 1
        if (path->header.stamp == goal_timestamp_) {
            requested_path_ = path;
            // reset to 0 to signalise, that there is no outstanding path
            goal_timestamp_ = ros::Time(0);
        }
        // else: drop this path (= do nothing)
    }
}

void PathController::followPathDoneCB(const actionlib::SimpleClientGoalState &state, const path_msgs::FollowPathResultConstPtr &result)
{
    ROS_INFO("Path execution finished.\n---------------------");

    path_msgs::NavigateToGoalResult nav_result;

    //TODO: do something more intelligent here :)
    nav_result.status = path_msgs::NavigateToGoalResult::STATUS_SUCCESS;

    navigate_to_goal_server_.setSucceeded(nav_result);
}

void PathController::followPathActiveCB()
{
    ROS_INFO("Path is now active.");
    // is there anything to do here?
}

void PathController::followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback)
{
    ROS_INFO("Got path executen feedback.");

    path_msgs::NavigateToGoalFeedback nav_feedback;
    //TODO: fill with something

    navigate_to_goal_server_.publishFeedback(nav_feedback);
}

void PathController::waitForPath(const geometry_msgs::PoseStamped &goal_pose)
{
    //TODO: Can there be concurrency problems? I think not, but better think a bit more deeply about it.

    goal_timestamp_ = goal_pose.header.stamp;
    goal_pub_.publish(goal_pose);

    while (!goal_timestamp_.isZero());
}
