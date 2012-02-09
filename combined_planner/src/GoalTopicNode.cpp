/**
 * @file GoalTopicNode.cpp
 * @date Feb 2012
 * @author marks
 */

// C/C++
#include <string>

// Project
#include "GoalTopicNode.h"

using namespace combined_planner;
using namespace actionlib;
using namespace move_base_msgs;
using namespace std;

GoalTopicNode::GoalTopicNode()
    : n_( "~" ),
      ac_( "move_base" )
{
    // Read config
    string goal_topic;
    n_.param<std::string>( "goal_topic", goal_topic, "/goal" );

    // Subscribe to the goal topic
    goal_subs_ = n_.subscribe<geometry_msgs::PoseStamped>
            ( goal_topic, 1, boost::bind( &GoalTopicNode::goalCB, this, _1 ));
}

void GoalTopicNode::goalCB( const geometry_msgs::PoseStampedConstPtr &goal )
{
    // Info
    ROS_INFO( "Received a new goal (%f %f)",
              goal->pose.position.x,
              goal->pose.position.y );

    // Cancel all previous goals
    ac_.cancelAllGoals();

    // Wait for action server
    if ( !ac_.waitForServer( ros::Duration( 1.0 ))) {
        ROS_ERROR( "No connection to move_base action server." );
        return;
    }

    // Send new goal
    ROS_INFO( "Sending move_base action goal." );
    MoveBaseGoal ac_goal;
    ac_goal.target_pose = *goal;
    ac_.sendGoal( ac_goal,
                  boost::bind( &GoalTopicNode::acDoneCB, this, _1, _2 ));
}

void GoalTopicNode::acDoneCB(const actionlib::SimpleClientGoalState &state,
                             const move_base_msgs::MoveBaseResultConstPtr &result ) {
    if ( state == SimpleClientGoalState::ABORTED
         || state == SimpleClientGoalState::PREEMPTED ) {
        ROS_WARN( "Move base action was aborted or preempted." );
    } else {
        ROS_INFO( "Move base action succeeded." );
    }
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "goal_topic_node" );
    GoalTopicNode node;
    ros::spin();

    return 0;
}


