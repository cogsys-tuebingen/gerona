/**
 * @file brain_node.cpp
 * @date July 2012
 * @author marks
 */

// C/C++
#include <sstream>
#include <stdlib.h>
#include <time.h>

// Workspace
#include <utils/LibRosUtil/RosMath.h>

// Project
#include "brain_node.h"

using namespace lib_ros_util;
using namespace std;
using namespace geometry_msgs;

namespace multiple_persons_brain {

BrainNode::BrainNode()
    : n_( "~" ),
      map_frame_( "/map" )
{
    // Number of person robots
    int num_persons = 0;
    n_.param( "num_persons", num_persons, 0 );
    ROS_ASSERT( num_persons > 0 );

    // Target points
    /*XmlRpc::XmlRpcValue target_list;
    n_.getParam( "targets", target_list );
    ROS_ASSERT( target_list.getType() == XmlRpc::XmlRpcValue::TypeArray );
    for ( int i = 0; i + 2 < target_list.size(); i += 2 ) {
        geometry_msgs::Point p;
        //p.x = target_list[i];
        //p.y = target_list[i];
        p.z = 0;
        ROS_INFO( "Target point %d: %f %f", i, p.x, p.y );
    }*/
    Point p;
    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    targets_.push_back( p );
    p.x = 6.0;
    p.y = -34.0;
    targets_.push_back( p );

    persons_.resize( 6 );
    persons_[0].idx = 1;
    persons_[0].target_pub = n_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple_robot_1/goal", 1, true );
    persons_[1].idx = 2;
    persons_[1].target_pub = n_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple_robot_2/goal", 1, true );
    persons_[2].idx = 3;
    persons_[2].target_pub = n_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple_robot_3/goal", 1, true );
    persons_[3].idx = 4;
    persons_[3].target_pub = n_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple_robot_4/goal", 1, true );
    persons_[4].idx = 5;
    persons_[4].target_pub = n_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple_robot_5/goal", 1, true );
    persons_[5].idx = 6;
    persons_[5].target_pub = n_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple_robot_6/goal", 1, true );

    // Create update timer
    update_timer_ = n_.createTimer( ros::Duration( 0.1 ), &BrainNode::updateCallback, this );

    // Random seed
    srand( time( NULL ));
}

void BrainNode::updateCallback( const ros::TimerEvent &e )
{
    Pose p;

    // For each simulated person
    for ( size_t i = 0; i < persons_.size(); ++i ) {
        // Get current position
        if ( !getRobotPose( persons_[i].idx, p ))
            continue;

        // Check distance to target
        if ( !persons_[i].has_target
             || RosMath::distance2d( p.position, persons_[i].target ) < 1.0 ) {
            // Select new target randomly
            selectAndPublishTarget( persons_[i], p );
        }
    }
}

void BrainNode::selectAndPublishTarget( Person &person, const geometry_msgs::Pose &p )
{
    PoseStamped new_target;
    int target_idx = rand() % targets_.size();
    new_target.pose.orientation = p.orientation;
    new_target.pose.position = targets_[ target_idx ];
    new_target.header.frame_id = map_frame_;
    new_target.header.stamp = ros::Time::now();
    person.target_pub.publish( new_target );
    person.target = new_target.pose.position;
    person.has_target = true;

    ROS_INFO( "Published new target. Idx: %d", person.idx );
}

bool BrainNode::getRobotPose( const int robot_idx, Pose &p )
{
    string robot_frame = getRobotName( robot_idx, "base_link" );
    tf::StampedTransform trafo;
    try {
        tf_.waitForTransform( robot_frame, map_frame_, ros::Time::now(), ros::Duration( 0.1 ));
        tf_.lookupTransform( map_frame_, robot_frame, ros::Time::now(), trafo );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR( "Cannot lookup transform. Reason: %s", ex.what());
        return false;
    }
    TransformStamped msg;
    tf::transformStampedTFToMsg( trafo, msg );
    p.position.x = msg.transform.translation.x;
    p.position.y = msg.transform.translation.y;
    p.position.z = 0;
    p.orientation = msg.transform.rotation;

    return true;
}

std::string BrainNode::getRobotName( const int robot_idx, const std::string &name )
{
    stringstream ss;
    ss << "/robot_" << robot_idx << "/" << name;
    return ss.str();
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
