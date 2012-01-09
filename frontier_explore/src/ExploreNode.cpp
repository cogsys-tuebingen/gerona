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

// OpenCv
#include <highgui.h>

// Project
#include "ExploreNode.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

ExploreNode::ExploreNode( ros::NodeHandle n ) :
    cvmap_( 10, 10, 0.1 ),
    visualize_( true )
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
    cvShowImage( "ExploreDebug", cvmap_.image );
    cvWaitKey( 100 );

    // Caluclate the frontiers
    ROS_INFO( "Calculation frontiers" );
    std::vector<geometry_msgs::Pose> frontiers;
    explorer_.getFrontiers( cvmap_, frontiers );
    ROS_INFO( "Found %d frontiers", (int)frontiers.size());

    // Visualize detected frontiers
    if ( visualize_ ) {
        ROS_INFO( "Publishing markers" );
        std::vector<visualization_msgs::Marker> markers;
        explorer_.getVisualizationMarkers( markers );
        for ( std::size_t i = 0; i < markers.size(); i++)
            visu_pub_.publish( markers[i] );
    }

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

int main( int argc, char* argv[] ) {

    ros::init(argc,argv, "frontier_explore");
    ros::NodeHandle n("~");
    ExploreNode explore_node( n );
    ros::Rate rate( 0.5 );

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
