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
      tf_( ros::Duration( 10.0 )),
      costmap_( "local_costmap", tf_ ),
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

void CombinedPlannerNode::update() {
    // Get new robot pose
    geometry_msgs::Pose robot_pose;
    if ( !getRobotPose( robot_pose, map_frame_id_ )) {
        return;
    }

    // Plan path to the next waypoint
    costmap_;
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
    /*map_->set( map->data, map->info.width, map->info.height );
    map_->setOrigin( Point2d( map->info.origin.position.x, map->info.origin.position.x ));
    map_frame_id_ = "/map"; //map->header.frame_id;*/

    // Create global planner on first map update
    if ( global_planner_ == NULL ) {
        global_planner_ = new GlobalPlanner( map_ );
    }

    // Ready to go
    got_map_ = true;
}

void CombinedPlannerNode::updateGoal( const geometry_msgs::PoseStampedConstPtr &goal )
{
    ROS_INFO("Got a new goal");
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
    std::vector<lib_path::Point2d> path_raw;
    global_planner_->getLatestPathRaw( path_raw );
    visualizePath( path_raw, "global_path_raw", 0, 0 );

    // Get & visualize flattened path
    global_planner_->getLatestPath( global_path_ );
    visualizePath( global_path_, "global_path", 1, 1 );

    // Calculate waypoints
    list<lib_path::Pose2d> waypoints;
    lib_path::Pose2d poseGoal, poseStart;
    poseGoal.x = pathEnd.x;
    poseGoal.y = pathEnd.y;
    poseGoal.theta = tf::getYaw( goal_map.pose.orientation );
    poseStart.x = robot_pose.position.x;
    poseStart.y = robot_pose.position.y;
    poseStart.theta = tf::getYaw( robot_pose.orientation );
    generateWaypoints( global_path_, poseGoal, waypoints );

    // Visualize waypoints
    visualizeWaypoints( waypoints, "waypoints", 3 );
}

void CombinedPlannerNode::generateWaypoints( const vector<lib_path::Point2d>& path,
                                             const lib_path::Pose2d& goal,
                                             list<lib_path::Pose2d>& waypoints ) const
{
    double min_waypoint_dist_ = 1.5;
    double max_waypoint_dist_ = 2.0;

    waypoints.clear();

    // Path too short? We need at least a start and an end point
    if ( path.size() < 2 ) {
        waypoints.push_back( goal );
        return;
    }

    // Compute waypoints
    double dist;
    double minus = 0.0;
    double step_size = min_waypoint_dist_; // 0.5*( min_waypoint_dist_ + max_waypoint_dist_ );
    lib_path::Pose2d dir( getNormalizedDelta( path[0], path[1] ));
    lib_path::Pose2d last;
    last.x = path[0].x; last.y = path[0].y;

    // For every point of the path except the first one
    for ( size_t i = 1; i < path.size(); ) {
        // Distance from last waypoint to next point of path
        dist = sqrt( pow( last.x - path[i].x, 2 ) + pow( last.y - path[i].y, 2 ));

        // If the distance is too short select next point of path
        if ( dist < max_waypoint_dist_ ) {
            // If this is the last point of the path, push back the goal pose
            if ( i >= path.size() - 1 ) {
                waypoints.push_back( goal );
                break;
            }

            // Compute new step vector. Index i is always less than path.size() - 1
            dir = getNormalizedDelta( path[i], path[i+1] );
            last.x = path[i].x;
            last.y = path[i].y;
            last.theta = dir.theta;
            ++i;

            // If the distance is greater than the minimum, add a new waypoint
            if ( dist >= min_waypoint_dist_ || minus >= min_waypoint_dist_ ) {
                waypoints.push_back( last );
                minus = 0.0;
                continue;
            } else {
                // Ensure that the next waypoint is not too far away
                minus += dist;
                continue;
            }
        }

        last.x += (step_size - minus) * dir.x;
        last.y += (step_size - minus) * dir.y;
        last.theta = dir.theta;
        minus = 0.0;
        waypoints.push_back( last );
    }
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
        const std::vector<lib_path::Point2d> &path,
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
    std::vector<lib_path::Point2d>::const_iterator it = path.begin();
    for ( ; it != path.end(); it++ ) {
        p.x = it->x;
        p.y = it->y;
        marker.points.push_back( p );
    }
    visu_pub_.publish( marker );
}

void CombinedPlannerNode::visualizeWaypoints( const list<lib_path::Pose2d> &wp, string ns, int id ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = ns;
    marker.color.g = 1.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 0.25f;
    marker.id = id;
    geometry_msgs::Point p;
    p.z = 0;
    std::list<lib_path::Pose2d>::const_iterator it = wp.begin();
    for ( ; it != wp.end(); it++ ) {
        p.x = it->x;
        p.y = it->y;
        marker.points.push_back( p );
        p.x += 0.8*cos( it->theta );
        p.y += 0.8*sin( it->theta );
        marker.points.push_back( p );
    }
    visu_pub_.publish( marker );
}

lib_path::Pose2d CombinedPlannerNode::getNormalizedDelta( const Point2d &start, const Point2d &end ) const
{
    lib_path::Pose2d result;
    result.x = end.x - start.x;
    result.y = end.y - start.y;
    double l = sqrt( result.x*result.x + result.y*result.y );
    result.x /= l;
    result.y /= l;
    result.theta = atan2( result.y, result.x );

    return result;
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "combined_planner" );
    CombinedPlannerNode node;

    ros::Rate rate( 20 );
    while ( ros::ok()) {
        ros::spinOnce();
        node.update();
        rate.sleep();
    }

    return 0;
}
