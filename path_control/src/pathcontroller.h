#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/FollowPathAction.h>
#include <path_msgs/PlanPathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>


/**
 * @brief This class is the interface to the user, who wants the robot to navigate.
 *
 * This class is the interface to the user. It receives the goal positon and manages path
 * planning and exection. Thus the user has not to communicate directly with planner and
 * follower.
 */
class PathController
{
public:
    PathController(ros::NodeHandle &nh);

private:
    typedef actionlib::SimpleActionServer<path_msgs::NavigateToGoalAction> NavToGoalServer;
    typedef actionlib::SimpleActionClient<path_msgs::FollowPathAction> FollowPathClient;
    typedef actionlib::SimpleActionClient<path_msgs::PlanPathAction> PlanPathClient;
    typedef actionlib::SimpleClientGoalState GoalState;

    struct Options {
        //! Maximum number of replanning attempts on some failure. If the path execution still failes after this number
        //! of replannings, the goal will be aborted.
        int num_replan_attempts;
    };

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    Options opt_;

    //! Action server that communicates with the high level control (the node that sends the goal points).
    NavToGoalServer navigate_to_goal_server_;

    //! Action client to communicate with the path_follower package.
    FollowPathClient follow_path_client_;

    //! Action clients to communicate with the path_planner package
    std::map<std::string, boost::shared_ptr<PlanPathClient>> path_planner_client_;

    //! Publisher for text to speech messages.
    ros::Publisher speech_pub_;
    //! Publisher for visualizing the current goal
    ros::Publisher goal_pub_;

    nav_msgs::PathConstPtr requested_path_;

    //! Final state of the last finished follow_path action.
    actionlib::SimpleClientGoalState::StateEnum follow_path_final_state_;
    //! Result of the last finished follow_path action
    path_msgs::FollowPathResultConstPtr follow_path_result_;
    //! False if follow_path action is currently running, otherwise true.
    bool follow_path_done_;

    //! The goal, that is currently executed.
    path_msgs::NavigateToGoalGoalConstPtr current_goal_;




    //! Callback that receives the goal poses and initiates planning and following.
    void navToGoalActionCallback(const path_msgs::NavigateToGoalGoalConstPtr &goal);

    /**
     * @brief Handles the planning and path following
     *
     * This is the heart of the path controller. It sends the goal to the planner, waits for
     * the path and forwards the path to the follower. During this, it sends status reports to
     * the high level node who sent the goal.
     * This method blocks until path execution is finished or aborted.
     *
     * @return True, if path execution finishes successfully, false if it is aborted for some
     *         reason.
     */
    bool processGoal();

    //! Finishes the navigate_to_goal action and sends an appropriate result message.
    void handleFollowPathResult();

    //! Callback for result of finished FollowPathAction.
    void followPathDoneCB(const actionlib::SimpleClientGoalState &state,
                          const path_msgs::FollowPathResultConstPtr &result);

    //! Callback for FollowPathAction becoming active.
    void followPathActiveCB();

    //! Callback for FollowPathAction feedback.
    void followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback);

    void findPath();

    /**
     * @brief Blocks execution until current follow_path goal is finished or timeout expires
     * @param timeout
     * @return True if goal is finished, false if timeout expires before.
     */
    bool waitForFollowPathDone(ros::Duration timeout);

    //! Send 'text' to a text to speech processor.
    void say(std::string text);

    void publishGoalMessage();
};

#endif // PATHCONTROLLER_H
