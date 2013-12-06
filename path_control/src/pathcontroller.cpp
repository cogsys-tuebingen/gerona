#include "pathcontroller.h"

PathController::PathController(ros::NodeHandle &nh):
    node_handle_(nh),
    navigate_to_goal_server_(nh, "navigate_to_goal", boost::bind(&PathController::navToGoalActionCallback, this, _1), false),
    follow_path_client_("follow_path"),
    goal_timestamp_(ros::Time(0))
{
    ROS_INFO("Wait for follow_path action server...");
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
    ROS_INFO("Start Action!! [%d]", goal->debug_test);
    // ...

    //FIXME: uncomment after testing!
    waitForPath(goal->goal_pose);

    path_msgs::FollowPathGoal path_action_goal;
    path_action_goal.debug_test = goal->debug_test;
    //path_action_goal.path = *requested_path_;
    // ... set some more parameters here...

    follow_path_client_.sendGoal(path_action_goal,
                                 boost::bind(&PathController::followPathDoneCB, this, _1, _2),
                                 boost::bind(&PathController::followPathActiveCB, this),
                                 boost::bind(&PathController::followPathFeedbackCB, this, _1));

    while ( ! follow_path_client_.getState().isDone() ) {
        if (navigate_to_goal_server_.isPreemptRequested()) {
            ROS_INFO("Preemt goal [%d].\n---------------------", goal->debug_test);
            follow_path_client_.cancelGoal();
            navigate_to_goal_server_.setPreempted();
            break;
        }
        if (navigate_to_goal_server_.isNewGoalAvailable()) {
            ROS_INFO("New goal available [%d].\n---------------------", goal->debug_test);
            follow_path_client_.cancelGoal();
            //navigate_to_goal_server_.acceptNewGoal();
            navigate_to_goal_server_.setPreempted();
            break;
        }
    }

    // block this function, until the action is running
    //follow_path_client_.waitForResult(); //TODO: Timeout?
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
    ROS_INFO("Path execution finished [%d].\n---------------------", result->debug_test);

    path_msgs::NavigateToGoalResult nav_result;

    //TODO: do something more intelligent here :)
    nav_result.status = result->status; //path_msgs::NavigateToGoalResult::STATUS_SUCCESS;
    nav_result.debug_test = result->debug_test;


    switch (state.state_) {
    case GoalState::ABORTED:
        navigate_to_goal_server_.setAborted(nav_result);
        break;

    case GoalState::PREEMPTED:
        navigate_to_goal_server_.setPreempted(nav_result);
        break;

    case GoalState::SUCCEEDED:
    default: //TODO: Are there other states, that should _not_ handled like SUCCEEDED?
        navigate_to_goal_server_.setSucceeded(nav_result);
        break;
    }
}

void PathController::followPathActiveCB()
{
    ROS_INFO("Path is now active.");
    // is there anything to do here?
}

void PathController::followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback)
{
    ROS_INFO("Got path executen feedback [%d].", feedback->debug_test);

    path_msgs::NavigateToGoalFeedback nav_feedback;
    //TODO: fill with something usefull
    nav_feedback.debug_test = feedback->debug_test;

    navigate_to_goal_server_.publishFeedback(nav_feedback);
}

void PathController::waitForPath(const geometry_msgs::PoseStamped &goal_pose)
{
    //TODO: Can there be concurrency problems? I think not, but better think a bit more deeply about it.

    goal_timestamp_ = goal_pose.header.stamp;
    goal_pub_.publish(goal_pose);

    ROS_DEBUG("Wait for path...");
    while (!goal_timestamp_.isZero()
           && ros::ok()
           && !navigate_to_goal_server_.isPreemptRequested()
           && !navigate_to_goal_server_.isNewGoalAvailable())
    {}
    ROS_DEBUG("Stop waiting (stamp: %d;   ok: %d;   preempt: %d;   new goal: %d)",
              !goal_timestamp_.isZero(),
              ros::ok(),
              !navigate_to_goal_server_.isPreemptRequested(),
              !navigate_to_goal_server_.isNewGoalAvailable());
}
