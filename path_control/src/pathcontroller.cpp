#include "pathcontroller.h"

PathController::PathController(ros::NodeHandle &nh):
    node_handle_(nh),
    navigate_to_goal_server_(nh, "navigate_to_goal", boost::bind(&PathController::navToGoalActionCallback, this, _1), false),
    follow_path_client_("follow_path"),
    goal_timestamp_(ros::Time(0)),
    unexpected_path_(false)
{
    ROS_INFO("Wait for follow_path action server...");
    follow_path_client_.waitForServer();

    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 0);

    navigate_to_goal_server_.start();

    path_sub_ = nh.subscribe<nav_msgs::Path>("/path", 10, &PathController::pathCallback, this);

    ROS_INFO("Initialisation done.");
}

void PathController::navToGoalActionCallback(const path_msgs::NavigateToGoalGoalConstPtr &goal)
{
    ROS_INFO("Start Action!! [%d]", goal->debug_test);

    if (unexpected_path_) {
        ROS_INFO("Cancel execution of unexpected path.");
        follow_path_client_.cancelGoal();
    }

    follow_path_done_ = false;

    // ...

    waitForPath(goal->goal_pose);

    // before we're continuing, check if the goal already has been preemted to avoid unnecessary start of follow_path
    // action
    if (navigate_to_goal_server_.isPreemptRequested()) {
        ROS_INFO("Preempt goal [%d].\n---------------------", goal->debug_test);
        navigate_to_goal_server_.setPreempted();
        return;
    }

    path_msgs::FollowPathGoal path_action_goal;
    path_action_goal.debug_test = goal->debug_test;
    path_action_goal.path = *requested_path_;
    path_action_goal.velocity = 0.5; //FIXME: make this configurable (send speed from highlevel)
    //TODO: ... set some more parameters here?...

    follow_path_client_.sendGoal(path_action_goal,
                                 boost::bind(&PathController::followPathDoneCB, this, _1, _2),
                                 boost::bind(&PathController::followPathActiveCB, this),
                                 boost::bind(&PathController::followPathFeedbackCB, this, _1));

    while ( ! follow_path_client_.getState().isDone() ) {
        if (navigate_to_goal_server_.isPreemptRequested()) {
            ROS_INFO("Preempt goal [%d].\n---------------------", goal->debug_test);
            follow_path_client_.cancelGoal();
            // wait until the goal is really canceled (= done callback is called).
            if (!waitForFollowPathDone(ros::Duration(10))) {
                ROS_WARN("follow_path_client does not react to cancelGoal() for 10 seconds.");
            }

            navigate_to_goal_server_.setPreempted();

            // don't check for new goal here. If there is one, it will cause a new execution of this callback, after
            // this instance has stopped.
            return;
        }

        // As long as only one action client is active, a new goal should automatically preempt the former goal.
        // Separately checking for new goals should only be necessary, if there are more than one clients (or a client
        // that gets restarted), which is currently not intended.
//        if (navigate_to_goal_server_.isNewGoalAvailable()) {
//            ROS_INFO("New goal available [%d].\n---------------------", goal->debug_test);
//            follow_path_client_.cancelGoal();
//            navigate_to_goal_server_.setPreempted();
//            break;
//        }
    }

    handleFollowPathResult();
}

void PathController::handleFollowPathResult()
{
    /*** IMPORTANT: No matter, what the result is, the navigate_to_goal action has to be finished in some way! ***/

    // wait until the action is really finished
    if (!waitForFollowPathDone(ros::Duration(10))) {
        ROS_WARN("Wait for follow_path action to be finished, but timeout expired!");
        navigate_to_goal_server_.setAborted(path_msgs::NavigateToGoalResult(), "Wait for follow_path action to be finished, but timeout expired.");
        return;
    }


    /// Construct result message
    path_msgs::NavigateToGoalResult nav_result;

    //TODO: do something more intelligent here :)
    nav_result.status = follow_path_result_->status;
    nav_result.debug_test = follow_path_result_->debug_test;


    /* Terminate navigate_to_goal action according to the final state of the the follow_path action.
     *
     * According to [1] only REJECTED, RECALLED, PREEMPTED, ABORTED and SUCCEEDED are terminal states.
     * Thus theses states should be the only ones, that can occur here.
     *
     * [1] http://wiki.ros.org/actionlib/DetailedDescription
     */
    switch (follow_path_final_state_) {
    case GoalState::REJECTED:
    case GoalState::RECALLED:
    case GoalState::ABORTED:
        navigate_to_goal_server_.setAborted(nav_result);
        break;

    case GoalState::PREEMPTED:
        // This should never happen, because this method should not be called when the goal was preemted (this is
        // handled separately in the execute-callback).
        ROS_ERROR("This function should never receive a preemted goal. This is likely a bug! [file %s, line %d]",
                  __FILE__, __LINE__);
        navigate_to_goal_server_.setAborted(nav_result);
        break;

    case GoalState::SUCCEEDED:
        navigate_to_goal_server_.setSucceeded(nav_result);
        break;

    default: //TODO: Are there other states, that should be handled somehow?
        ROS_ERROR("Unexpected final state of follow_path goal. navigate_to_goal is aborted. Maybe this is a bug. [file %s, line %d]",
                  __FILE__, __LINE__);
        navigate_to_goal_server_.setAborted(nav_result);

    }
}

void PathController::pathCallback(const nav_msgs::PathConstPtr &path)
{
    if (goal_timestamp_.isZero()) { // unexpected path -> case 2

        // unexpected paths are not allowed to preempt regular action-based goals
        if (!navigate_to_goal_server_.isActive()) {
            ROS_INFO("Execute unexpected path.");
            unexpected_path_ = true;

            path_msgs::FollowPathGoal path_action_goal;
            path_action_goal.debug_test = 255;
            path_action_goal.path = *path;
            path_action_goal.velocity = 0.5; //FIXME: make this configurable

            // only simple callback that resets unexpected_path_, feedback is ignored.
            follow_path_client_.sendGoal(path_action_goal,
                                         boost::bind(&PathController::followUnexpectedPathDoneCB, this, _1, _2));
        } else {
            ROS_DEBUG("Unexpected path omitted.");
        }

    } else { // expected path -> case 1

        if (path->header.stamp == goal_timestamp_) {
            requested_path_ = path;
            // reset to 0 to signalise, that there is no outstanding path
            goal_timestamp_ = ros::Time(0);
        }
        // else: drop this path (= do nothing)
    }
}

void PathController::followPathDoneCB(const actionlib::SimpleClientGoalState &state,
                                      const path_msgs::FollowPathResultConstPtr &result)
{
    ROS_INFO("Path execution finished [%d].\n---------------------", result->debug_test);

    follow_path_final_state_ = state.state_;
    follow_path_result_ = result;
    follow_path_done_ = true;
}

void PathController::followPathActiveCB()
{
    ROS_INFO("Path is now active.");
    // is there anything to do here?
}

void PathController::followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback)
{
    ROS_INFO_THROTTLE(1,"Driven distance: %g;  Distance to goal: %g", feedback->dist_driven, feedback->dist_goal);

    path_msgs::NavigateToGoalFeedback nav_feedback;
    //TODO: fill with something usefull
    nav_feedback.debug_test = feedback->debug_test;

    navigate_to_goal_server_.publishFeedback(nav_feedback);
}

void PathController::followUnexpectedPathDoneCB(const actionlib::SimpleClientGoalState &state,
                                                const path_msgs::FollowPathResultConstPtr &result)
{
    ROS_INFO("Execution of unexpected path finished [%d, %s].\n---------------------",
             result->debug_test,state.toString().c_str());
    unexpected_path_ = false;
}

void PathController::waitForPath(const geometry_msgs::PoseStamped &goal_pose)
{
    //TODO: Can there be concurrency problems? I think not, but better think a bit more deeply about it.

    //TODO: Timeout? Not so urgent here, as a new goal will abort waiting.
    //TODO: Necessary to check for new goals? - I think yes.

    goal_timestamp_ = goal_pose.header.stamp;
    goal_pub_.publish(goal_pose);

    ROS_DEBUG("Wait for path...");
    while (!goal_timestamp_.isZero()
           && ros::ok()
           && !navigate_to_goal_server_.isPreemptRequested()
           && !navigate_to_goal_server_.isNewGoalAvailable())
    { }
    ROS_DEBUG("Stop waiting (stamp: %d;   ok: %d;   preempt: %d;   new goal: %d)",
              !goal_timestamp_.isZero(),
              ros::ok(),
              !navigate_to_goal_server_.isPreemptRequested(),
              !navigate_to_goal_server_.isNewGoalAvailable());
}

bool PathController::waitForFollowPathDone(ros::Duration timeout)
{
    ros::Time expire_time = ros::Time::now() + timeout;

    while (!follow_path_done_ && expire_time > ros::Time::now());

    return follow_path_done_;
}
