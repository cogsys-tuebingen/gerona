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


/**
 * @brief The PathController class
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
        //! Velocity which is used for unexpected paths (which come without action and thus have no specified velocity).
        float unexpected_path_velocity;

        //! Maximum number of replanning attempts on some failure. If the path execution still failes after this number
        //! of replannings, the goal will be aborted.
        int num_replan_attempts;
    };

    ros::NodeHandle node_handle_;

    Options opt_;

    //! Action server that communicates with the high level control (the node that sends the goal points).
    NavToGoalServer navigate_to_goal_server_;

    //! Action client to communicate with the path_follower package.
    FollowPathClient follow_path_client_;

    //! Subscribes to a goal position
    ros::Subscriber goal_sub_;

    //! Action client to communicate with the path_planner package
    PlanPathClient path_planner_client_;

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

    //! Final state of the last finished follow_path action.
    actionlib::SimpleClientGoalState::StateEnum follow_path_final_state_;
    //! Result of the last finished follow_path action
    path_msgs::FollowPathResultConstPtr follow_path_result_;
    //! False if follow_path action is currently running, otherwise true.
    bool follow_path_done_;

    //! True, if the currently executed path was unexpected.
    bool unexpected_path_;

    //! The goal, that is currently executed.
    path_msgs::NavigateToGoalGoalConstPtr current_goal_;




    void navToGoalActionCallback(const path_msgs::NavigateToGoalGoalConstPtr &goal);

    bool processGoal();

    void handleFollowPathResult();

    //! Callback for the paths published e.g. by path_planner.
    void pathCallback(const nav_msgs::PathConstPtr &path);

    //! Callback for result of finished FollowPathAction.
    void followPathDoneCB(const actionlib::SimpleClientGoalState &state,
                          const path_msgs::FollowPathResultConstPtr &result);

    //! Callback for FollowPathAction becoming active.
    void followPathActiveCB();

    //! Callback for FollowPathAction feedback.
    void followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback);

    //! Callback for result of finished FollowPathAction with an unexpected path.
    void followUnexpectedPathDoneCB(const actionlib::SimpleClientGoalState &state,
                                    const path_msgs::FollowPathResultConstPtr &result);

    //! Send a goal pose to path_follower and wait for the resulting path.
    /** \todo Timeout! */
    void waitForPath(const geometry_msgs::PoseStamped &goal_pose);

    void findPath(const geometry_msgs::PoseStamped &goal);

    /**
     * @brief Blocks execution until current follow_path goal is finished or timeout expires
     * @param timeout
     * @return True if goal is finished, false if timeout expires before.
     */
    bool waitForFollowPathDone(ros::Duration timeout);
};

#endif // PATHCONTROLLER_H
