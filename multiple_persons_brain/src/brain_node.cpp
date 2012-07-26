/**
 * @file brain_node.cpp
 * @date July 2012
 * @author marks
 */


// Project
#include "brain_node.h"

namespace multiple_persons_brain {

BrainNode::BrainNode()
{
    // Read config
    ros::NodeHandle n( "~" );

    // Number of person robots
    int num_persons = 0;
    n.param( "num_persons", num_persons, 0 );
    if ( num_persons <= 0 ) {
        ROS_FATAL( "Number of controlled person robots <= 0. Check param \"num_persons\"" );
        exit( -1 );
    }

    // Target points
    // TODO

    // Create update timer
    update_timer_ = n.createTimer( ros::Duration( 0.1 ), &BrainNode::updateCallback, this );
}

void BrainNode::updateCallback( const ros::TimerEvent &e )
{
    ROS_INFO( "Update" );
}

void BrainNode::spin()
{
    ros::spin();
}

} // namespace

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "brain_node" );
    multiple_persons_brain::BrainNode node;
    node.spin();
    return 0;
}
