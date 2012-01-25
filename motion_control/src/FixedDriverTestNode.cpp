
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_control/MotionAction.h>

#include <cmath>

int main( int argc, char* argv[] )
{
    // Init
    ros::init( argc, argv, "FixedDriverTestNode" );
    actionlib::SimpleActionClient<motion_control::MotionAction> ac( "motion_control" );
    ROS_INFO( "Waiting for action server" );
    ac.waitForServer();

    // Send goal
    motion_control::MotionGoal goal;
    goal.deltaf = 20.0*M_PI/180.0;
    goal.v = 0.6;
    goal.mode = motion_control::MotionGoal::MOTION_FIXED_PARAMS;
    ROS_INFO( "Sending goal" );
    ac.sendGoal( goal );
    ROS_INFO( "Waiting for result" );
    ac.waitForResult();
    ROS_INFO( "Done" );
}
