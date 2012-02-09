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
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

namespace combined_planner {

class GoalTopicNode
{
public:
    GoalTopicNode();

    void goalCB( const geometry_msgs::PoseStampedConstPtr& goal );

    void acDoneCB( const actionlib::SimpleClientGoalState& state,
                   const move_base_msgs::MoveBaseResultConstPtr& result );

private:
    /// The ROS handle
    ros::NodeHandle n_;

    /// Action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    /// Used to get new goals
    ros::Subscriber goal_subs_;
};

} // namespace

#endif // GOALTOPICNODE_H
