/**
 * @file CombinedPlannerNode.cpp
 * @date Jan 2012
 * @author marks
 */

// C/C++
#include <cmath>

// ROS
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/Marker.h>

// Workspace
#include <utils/LibUtil/Stopwatch.h>

// Project
#include "CombinedPlannerNode.h"


using namespace lib_path;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class CombinedPlannerNode
///////////////////////////////////////////////////////////////////////////////


CombinedPlannerNode::CombinedPlannerNode()
    : n_( "~" ),
      tf_( ros::Duration( 5.0 )),
      map_( NULL ),
      global_planner_( NULL ),
      got_map_( false )
{
    // Topic names parameters
    n_.param<std::string>( "map_topic", map_topic_, "/map_inflated" );
    n_.param<std::string>( "goal_topic", goal_topic_, "/goal" );
    n_.param<std::string>( "path_topic", path_topic_, "/a_star_path" );

    // Subscribe
    map_subs_ = n_.subscribe<nav_msgs::OccupancyGrid>( map_topic_, 1, boost::bind( &CombinedPlannerNode::updateMap, this, _1 ));
    goal_subs_ = n_.subscribe<geometry_msgs::PoseStamped>( goal_topic_, 1, boost::bind( &CombinedPlannerNode::updateGoal, this, _1 ));

    // Advertise
    path_pub_ = n_.advertise<nav_msgs::Path>( path_topic_, 5 );
    visu_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_markers", 5 );
}

void CombinedPlannerNode::updateMap( const nav_msgs::OccupancyGridConstPtr &map )
{
    // Create map?
    if ( map_ == NULL ) {
        map_ = new SimpleGridMap2d( map->info.width, map->info.height, map->info.resolution );
        ROS_INFO( "Received first map: Width %d, height %d, resolution %f",
                  map->info.width, map->info.height, map->info.resolution );
    }

    // Map setup
    map_->set( map->data, map->info.width, map->info.height );
    map_->setOrigin( Point2d( map->info.origin.position.x, map->info.origin.position.x ));
    map_frame_id_ = "/map"; //map->header.frame_id;

    // Create global planner on first map update
    if ( global_planner_ == NULL ) {
        global_planner_ = new GlobalPlanner( map_ );
    }

    // Ready to go
    got_map_ = true;
}

void CombinedPlannerNode::updateGoal( const geometry_msgs::PoseStampedConstPtr &goal )
{
    global_path_.clear();
    if ( !got_map_ ) {
        ROS_WARN( "Got a goal but no map. It's not possible to plan a path!" );
        return;
    }

    // Convert goal int map coordinate system if necessary
    geometry_msgs::PoseStamped goal_map;
    if ( goal->header.frame_id.compare( map_frame_id_ ) != 0 ) {
        try {
            tf_.transformPose( map_frame_id_, *goal, goal_map );
        } catch (tf::TransformException& ex) {
            ROS_ERROR( "Cannot transform goal into map coordinates. Reason: %s",
                       ex.what());
            return;
        }
    } else {
        goal_map = *goal;
    }

    // Get the current robot pose in map coordinates
    geometry_msgs::Pose robot_pose;
    if (!getRobotPose( robot_pose, map_frame_id_ ))
        return;

    // Run global planner
    lib_path::Point2d pathStart( robot_pose.position.x, robot_pose.position.y );
    lib_path::Point2d pathEnd( goal_map.pose.position.x, goal_map.pose.position.y );
    if ( !global_planner_->planPath( pathStart, pathEnd )) {
        ROS_WARN( "No global path found." );
        return;
    }

    // Visualize raw path
    std::list<lib_path::Point2d> path_raw;
    global_planner_->getLatestPathRaw( path_raw );
    visualizePath( path_raw, "global_path_raw", 0, 0 );

    // Get & visualize flattened path
    global_planner_->getLatestPath( global_path_ );
    visualizePath( global_path_, "global_path", 1, 1 );
}

bool CombinedPlannerNode::selectNextWaypoint( const lib_path::Pose2d &robot_pose,
                                              lib_path::Point2d& next ) const
{
    /*if ( global_path_.empty())
        return false;

    // Check distance to next waypoint
    double dist = 0;
    lib_path::Pose2d wp = global_path_.front();
    double dist = sqrt( pow( wp.x - robot_pose.x, 2 ) + pow( wp.y - robot_pose.y, 2 ));
    if ( dist < 2.0 ) {

    }*/

}

bool CombinedPlannerNode::getRobotPose( geometry_msgs::Pose& pose, const std::string& map_frame )
{
    tf::StampedTransform trafo;
    geometry_msgs::TransformStamped msg;
    try {
        tf_.lookupTransform( map_frame.c_str(), "/base_link", ros::Time(0), trafo );
    } catch (tf::TransformException& ex) {
        ROS_ERROR( "Error getting the robot position. Reason: %s",
                   ex.what());
        return false;
    }

    tf::transformStampedTFToMsg( trafo, msg );
    pose.position.x = msg.transform.translation.x;
    pose.position.y = msg.transform.translation.y;
    pose.position.z = 0;
    pose.orientation = msg.transform.rotation;

    return true;
}

void CombinedPlannerNode::visualizePath(
        const std::list<lib_path::Point2d> &path,
        const std::string& ns,
        const int color,
        const int id )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = ns;
    if ( color != 1 )
        marker.color.r = 1.0f;
    else
        marker.color.b = 1.0f;
    marker.color.a = 0.75f;
    marker.scale.x = marker.scale.y = 0.1f;
    marker.id = id;
    geometry_msgs::Point p;
    p.z = 0;
    std::list<lib_path::Point2d>::const_iterator it = path.begin();
    for ( ; it != path.end(); it++ ) {
        p.x = it->x;
        p.y = it->y;
        marker.points.push_back( p );
    }
    visu_pub_.publish( marker );
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "combined_planner" );
    CombinedPlannerNode node;

    ros::Rate rate( 100 );
    while ( ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
