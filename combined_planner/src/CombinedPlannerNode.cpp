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
using namespace lib_ros_util;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class CombinedPlannerNode
///////////////////////////////////////////////////////////////////////////////


CombinedPlannerNode::CombinedPlannerNode()
    : n_( "~" ),
      tf_( ros::Duration( 10.0 )),
      lmap_ros_( "local_costmap", tf_ ),
      lmap_wrapper_( &lmap_cpy_ ),
      gmap_wrapper_( &gmap_cpy_ ),
      global_planner_( NULL ),
      local_planner_( NULL ),
      got_map_( false )
{
    // Topic names parameters
    n_.param<std::string>( "map_topic", map_topic_, "/map_inflated" );
    n_.param<std::string>( "goal_topic", goal_topic_, "/goal" );
    n_.param<std::string>( "path_topic", path_topic_, "/a_star_path" );

    // Member
    ros::NodeHandle rs_n( "~/reed_shepp/" );
    local_planner_ = new LocalPlanner( rs_n );

    // Subscribe
    map_subs_ = n_.subscribe<nav_msgs::OccupancyGrid>( map_topic_, 1, boost::bind( &CombinedPlannerNode::updateMap, this, _1 ));
    goal_subs_ = n_.subscribe<geometry_msgs::PoseStamped>( goal_topic_, 1, boost::bind( &CombinedPlannerNode::updateGoal, this, _1 ));

    // Advertise
    path_pub_ = n_.advertise<nav_msgs::Path>( path_topic_, 5 );
    visu_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_markers", 5 );
}

void CombinedPlannerNode::update( bool force_replan )
{
    // Get new robot pose
    geometry_msgs::Pose robot_pose;
    if ( waypoints_.empty() || !getRobotPose( robot_pose, map_frame_id_ )) {
        return; // No pose or no waypoint left
    }

    // No waypoints left except of the goal? Check if the goal is reached.
    lib_path::Pose2d start;
    start.x = robot_pose.position.x;
    start.y = robot_pose.position.y;
    start.theta = tf::getYaw( robot_pose.orientation );
    if ( waypoints_.size() == 1 && isGoalReached( start, waypoints_.front())) {
        waypoints_.clear(); // Finished
        visualizeWaypoints( waypoints_ , "waypoints", 3 );
        publishEmptyLocalPath();
        ROS_INFO( "Goal reached" );
        return;
    }

    // Select next waypoint
    bool new_wp = false;
    while ( waypoints_.size() > 1 && nextWaypoint( start )) {
        waypoints_.pop_front();
        new_wp = true;
    }

    // New waypoint selected?
    if ( !force_replan && !new_wp )
        return;

    // Publish waypoints
    ROS_INFO( "Selected new waypoint." );
    visualizeWaypoints( waypoints_, "waypoints", 3 );

    // Plan path to the next waypoint
    lmap_ros_.getCostmapCopy( lmap_cpy_ );
    local_planner_->setMap( &lmap_wrapper_ );
    if ( local_planner_->planPath( start, waypoints_.front())) {
        std::list<lib_path::Pose2d> lpath;
        local_planner_->getPath( lpath );
        publishLocalPath( lpath );
    }
}

bool CombinedPlannerNode::nextWaypoint( const lib_path::Pose2d &robot_pose ) const
{
    if ( waypoints_.empty())
        return false;

    Pose2d next = waypoints_.front();
    double d_dist, d_theta;
    getPoseDelta( robot_pose, next, d_dist, d_theta );

    return d_dist < 1.0 && d_theta < 0.25*M_PI;
}

bool CombinedPlannerNode::isGoalReached( const lib_path::Pose2d& robot_pose,
                                         const lib_path::Pose2d &goal ) const
{
    double d_dist, d_theta;
    getPoseDelta( robot_pose, goal, d_dist, d_theta );
    return d_dist < 0.1 && d_theta < 0.1*M_PI;
}

void CombinedPlannerNode::getPoseDelta( const lib_path::Pose2d &p, const lib_path::Pose2d &q,
                                       double &d_dist, double &d_theta ) const
{
    d_dist = sqrt( pow( p.x - q.x, 2 ) + pow( p.y - q.y, 2 ));
    d_theta = fabs( p.theta - q.theta );
    while ( d_theta > M_PI ) /// @todo use lib utils
        d_theta -= 2.0*M_PI;
    d_theta = fabs( d_theta );
}

void CombinedPlannerNode::updateMap( const nav_msgs::OccupancyGridConstPtr &map )
{
    gmap_cpy_ = *map;

    // Create global planner on first map update
    if ( global_planner_ == NULL ) {
        global_planner_ = new GlobalPlanner( &gmap_wrapper_ );
    }
    map_frame_id_ = "/map";

    // Ready to go
    got_map_ = true;
}

void CombinedPlannerNode::updateGoal( const geometry_msgs::PoseStampedConstPtr &goal )
{
    ROS_INFO("Got a new goal");
    waypoints_.clear();
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
    path_raw.clear();
    global_planner_->getLatestPath( path_raw );
    visualizePath( path_raw, "global_path", 1, 1 );

    // Calculate waypoints
    lib_path::Pose2d poseGoal, poseStart;
    poseGoal.x = pathEnd.x;
    poseGoal.y = pathEnd.y;
    poseGoal.theta = tf::getYaw( goal_map.pose.orientation );
    poseStart.x = robot_pose.position.x;
    poseStart.y = robot_pose.position.y;
    poseStart.theta = tf::getYaw( robot_pose.orientation );
    generateWaypoints( path_raw, poseGoal, waypoints_ );

    // Visualize waypoints
    visualizeWaypoints( waypoints_, "waypoints", 3 );

    // Run an update
    update( true );
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

void CombinedPlannerNode::publishLocalPath( std::list<lib_path::Pose2d> &path )
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_frame_id_;
    path_msg.header.stamp = ros::Time::now();

    std::list<lib_path::Pose2d>::const_iterator it = path.begin();
    geometry_msgs::PoseStamped pose;
    for ( ; it != path.end(); it++ ) {
        pose.pose.position.x = it->x;
        pose.pose.position.y = it->y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw( it->theta );
        path_msg.poses.push_back( pose );
    }
    path_pub_.publish( path_msg );
}

void CombinedPlannerNode::publishEmptyLocalPath()
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_frame_id_;
    path_msg.header.stamp = ros::Time::now();
    path_pub_.publish( path_msg );
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

    ros::Rate rate( 10 );
    while ( ros::ok()) {
        ros::spinOnce();
        node.update();
        rate.sleep();
    }

    return 0;
}
