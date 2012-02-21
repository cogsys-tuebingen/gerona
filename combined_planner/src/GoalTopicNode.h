/**
 * @file GoalTopicNode.h
 * @date Feb 2012
 * @author marks
 */

#ifndef GOALTOPICNODE_H
#define GOALTOPICNODE_H

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

// Project
#include <combined_planner/GoToAction.h>

namespace combined_planner {

class GoalTopicNode
{
public:
    GoalTopicNode();

    void goalCB( const geometry_msgs::PoseStampedConstPtr& goal );

    void acDoneCB( const actionlib::SimpleClientGoalState& state,
                   const GoToResultConstPtr &result );

private:
    /// The ROS handle
    ros::NodeHandle n_;

    /// Action client
    actionlib::SimpleActionClient<GoToAction> ac_;

    /// Used to get new goals
    ros::Subscriber goal_subs_;

    /// Maximum robot speed m/s
    double max_speed_;
};

} // namespace

#endif // GOALTOPICNODE_H
