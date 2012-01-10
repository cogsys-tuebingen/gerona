/*
 * ExploreNode.cpp
 *
 * Created: Jan 2012
 * Author: Marks
 *
 */

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// ROS
#include <visualization_msgs/Marker.h>

// OpenCv
#include <highgui.h>

// Project
#include "ExploreNode.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

ExploreNode::ExploreNode( ros::NodeHandle n ) :
    cvmap_( 10, 10, 0.1 ),
    tf_( ros::Duration( 5.0 )),
    visualize_( true ),
    goals_marker_count_( 0 )
{
    map_service_client_ = n.serviceClient<nav_msgs::GetMap>( "/dynamic_map" );
    visu_pub_ = n.advertise<visualization_msgs::Marker>( "visualization_markers", 100 );
    cvNamedWindow("ExploreDebug", CV_WINDOW_NORMAL );
}

bool ExploreNode::calculateFrontiers() {
    // Request the current map
    ROS_INFO( "Requesting map" );
    nav_msgs::GetMap srv_map;
    if ( !map_service_client_.call( srv_map )) {
        ROS_ERROR( "Error requesting the map!" );
        return false;
    }

    // Create cv image
    mapToCvMap( srv_map.response.map, cvmap_ );
    cvmap_.erode( 2 );
    //cvShowImage( "ExploreDebug", cvmap_.image );
    //cvWaitKey( 100 );

    // Get the robot position
    geometry_msgs::Pose robot_pose;
    if ( !getRobotPose( robot_pose )) {
        ROS_ERROR( "Cannot get robot position." );
        return false;
    }

    // Caluclate the frontiers
    ROS_INFO( "Calculation exploration goals" );
    std::vector<geometry_msgs::Pose> goals;
    explorer_.getExplorationGoals( cvmap_, robot_pose, goals, 1.0, 5.0, 1.0);

    // Visualize detected frontiers
    if ( visualize_ ) {
        ROS_INFO( "Publishing markers" );
        publishGoalsVisu( goals );
    }

    return true;
}

bool ExploreNode::getRobotPose( geometry_msgs::Pose& pose ) {
    tf::StampedTransform trafo;
    geometry_msgs::TransformStamped msg;
    try {
        tf_.lookupTransform( "/map", "/base_link", ros::Time(0), trafo );
    } catch (tf::TransformException& ex) {
        return false;
    }

    tf::transformStampedTFToMsg( trafo, msg );
    pose.position.x = msg.transform.translation.x;
    pose.position.y = msg.transform.translation.y;
    pose.position.z = 0;
    pose.orientation = msg.transform.rotation;

    return true;
}

void ExploreNode::mapToCvMap( const nav_msgs::OccupancyGrid &map, CvMap& cvmap ) {
    // Check image size
    if ( cvmap.image->height != (int)map.info.height || cvmap.image->width != (int)map.info.width ) {
        cvmap.resize( map.info.width, map.info.height );
    }

    // Resolution/origin migth differ
    cvmap.resolution = map.info.resolution;
    cvmap.origin_x = map.info.origin.position.x;
    cvmap.origin_y = map.info.origin.position.y;

    // Copy map data
    for ( int i = 0; i < cvmap.image->imageSize; ++i ) {
        if ( map.data[i] == -1 )
            cvmap.data[i] = 128;
        else if ( map.data[i] < 10 )
            cvmap.data[i] = 255;
        else
            cvmap.data[i] = 0;
    }
}

void ExploreNode::publishGoalsVisu(const std::vector<geometry_msgs::Pose>& goals ) {
    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "exploration_goals";
    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 1.5;
    m.scale.y = 1.5;
    m.scale.z = 1.5;
    m.color.r = 255;
    m.color.g = 0;
    m.color.b = 0;
    m.color.a = 200;
    m.lifetime = ros::Duration(0);

    m.action = visualization_msgs::Marker::ADD;
    uint id = 0;
    unsigned int count = 0;
    for ( std::size_t i = 0; i < goals.size(); ++i ) {
        m.id = id;
        m.pose = goals[i];
        visu_pub_.publish( m );
        m.color.r = 0; // Only the first goal red
        m.color.b = 255;
        ++id;
        ++count;
    }

    m.action = visualization_msgs::Marker::DELETE;
    for (; id < goals_marker_count_; id++) {
        m.id = id;
        visu_pub_.publish( m );
    }
    goals_marker_count_ = count;
}

int main( int argc, char* argv[] ) {

    ros::init(argc,argv, "frontier_explore");
    ros::NodeHandle n("~");
    ExploreNode explore_node( n );
    ros::Rate rate( 2 );

    cvInitSystem( argc, argv );

    ros::spinOnce();
    //explore_node.calculateFrontiers();
    while ( ros::ok()) {
        explore_node.calculateFrontiers();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
