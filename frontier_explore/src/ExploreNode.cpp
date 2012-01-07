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

// Project
#include "ExploreNode.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

ExploreNode::ExploreNode( ros::NodeHandle n ) :
    visualize_( true )
{
    map_service_client_ = n.serviceClient<nav_msgs::GetMap>( "/dynamic_map" );
    visu_pub_ = n.advertise<visualization_msgs::Marker>( "visualization_markers", 100 );
}

bool ExploreNode::calculateFrontiers() {
    // Request the current map
    ROS_INFO( "Requesting map" );
    nav_msgs::GetMap srv_map;
    if ( !map_service_client_.call( srv_map )) {
        ROS_ERROR( "Error requesting the map!" );
        return false;
    }
    const nav_msgs::OccupancyGrid& map( srv_map.response.map );

    // Caluclate the frontiers
    ROS_INFO( "Calculation frontiers" );
    std::vector<geometry_msgs::Pose> frontiers;
    explorer_.getFrontiers( map, frontiers );
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

int main( int argc, char* argv[] ) {

    ros::init(argc,argv, "frontier_explore");
    ros::NodeHandle n("~");
    ExploreNode explore_node( n );
    ros::Rate rate( 0.5 );

    ros::spinOnce();
    while ( ros::ok()) {
        explore_node.calculateFrontiers();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
