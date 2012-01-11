/*
 * ExploreNode.cpp
 *
 * Created: Jan 2012
 * Author: Marks
 *
 */

// ROS
#include <visualization_msgs/Marker.h>

// OpenCv
#include <highgui.h>

// Project
#include "ExploreNode.h"

///////////////////////////////////////////////////////////////////////////////
// class ExploreNode
///////////////////////////////////////////////////////////////////////////////

ExploreNode::ExploreNode( ros::NodeHandle n ) :
    cvmap_( 10, 10, 0.1 ),
    tf_( ros::Duration( 5.0 )),
    goals_marker_count_( 0 )
{
    // Map options
    n.param<int>( "erode_iterations", erode_, 1 );
    n.param<int>( "downsample_iterations", downsample_, 1 );

    // Goal selection config
    double length_gain, ori_gain, path_gain, min_length;
    n.param<double>( "frontier_length_gain", length_gain, 1.0 );
    n.param<double>( "frontier_dist_gain", path_gain, 2.0 );
    n.param<double>( "frontier_orientation_gain", ori_gain, 5.0 );
    n.param<double>( "min_frontier_length", min_length, 0.5 );

    // Misc options
    n.param<bool>( "visualize", visualize_, false );
    n.param<bool>( "show_debug_map", show_debug_map_, false );

    // Check options
    if ( erode_ < 0 ) {
        ROS_WARN( "Invalid number of erode iterations: %d. Setting parameter \"erode_iterations\" to zero.", erode_ );
        erode_ = 0;
    }
    if ( downsample_ < 0 ) {
        ROS_WARN( "Invalid number of downsample iterations: %d. Setting parameter \"downsample_iterations\" to zero.", downsample_ );
        downsample_ = 0;
    }

    // Setup
    explorer_.setFrontierLengthGain( length_gain );
    explorer_.setOrientationChangeGain( ori_gain );
    explorer_.setPathLengthGain( path_gain );
    explorer_.setMinFrontierLength( min_length );

    // Neccessary ROS stuff
    map_service_client_ = n.serviceClient<nav_msgs::GetMap>( "/dynamic_map" );
    if ( visualize_ )
        visu_pub_ = n.advertise<visualization_msgs::Marker>( "visualization_markers", 100 );
    if ( show_debug_map_ )
        cvNamedWindow( "ExploreDebug", CV_WINDOW_NORMAL );
}

bool ExploreNode::calculateFrontiers() {
    // Request the current map
    nav_msgs::GetMap srv_map;
    if ( !map_service_client_.call( srv_map )) {
        ROS_ERROR( "Error requesting the map!" );
        return false;
    }

    // Create/process map
    occupancyGridToCvMap( srv_map.response.map, cvmap_ );
    cvmap_.erode( erode_ );
    for ( int i = 0; i < downsample_; ++i )
        cvmap_.downsample();

    // Debug image?
    if ( show_debug_map_ ) {
        cvShowImage( "ExploreDebug", cvmap_.image );
        cvWaitKey( 10 );
    }

    // Get the robot position
    geometry_msgs::Pose robot_pose;
    if ( !getRobotPose( robot_pose )) {
        ROS_ERROR( "Cannot get robot position." );
        return false;
    }

    // Caluclate the frontiers
    std::vector<geometry_msgs::Pose> goals;
    explorer_.getExplorationGoals( cvmap_, robot_pose, goals );

    // Visualize detected frontiers?
    if ( visualize_ )
        publishGoalsVisu( goals );

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

void ExploreNode::occupancyGridToCvMap( const nav_msgs::OccupancyGrid &map, CvMap& cvmap ) {
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
            cvmap.data[i] = 128; // No information
        else if ( map.data[i] < 10 )
            cvmap.data[i] = 255; // Open
        else
            cvmap.data[i] = 0; // Lethal obstacle
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
    while ( ros::ok()) {
        explore_node.calculateFrontiers();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
