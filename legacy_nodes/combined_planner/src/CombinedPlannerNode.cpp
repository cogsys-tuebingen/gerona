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
#include <utils/LibUtil/Stopwatch.h>

// Project
#include "PlannerExceptions.h"
#include "CombinedPlannerNode.h"

using namespace lib_path;
using namespace lib_ros_util;
using namespace std;
using namespace combined_planner;
using namespace actionlib;

///////////////////////////////////////////////////////////////////////////////
// class CombinedPlannerNode
///////////////////////////////////////////////////////////////////////////////


CombinedPlannerNode::CombinedPlannerNode()
    : SimpleActionServer<GoToAction>( "go_to", false ),
      n_( "~" ),
      tf_( ros::Duration( 10.0 )),
      lmap_ros_( "local_costmap", tf_ ),
      lmap_wrapper_( &lmap_cpy_ ),
      gmap_wrapper_( &gmap_cpy_ ),
      got_map_( false ),
      motion_ac_( "motion_control" )
{
    // Topic name parameters
    n_.param<std::string>( "map_topic", map_topic_, "/map_inflated" );
    n_.param<std::string>( "goal_topic", goal_topic_, "/goal" );
    n_.param<std::string>( "path_topic", path_topic_, "/path" );

    lmap_wrapper_.setLowerThreshold( 128 );
    lmap_wrapper_.setUpperThreshold( 254 );

    // Subscribe
    map_subs_ = n_.subscribe<nav_msgs::OccupancyGrid>( map_topic_, 1, boost::bind( &CombinedPlannerNode::globalMapCB, this, _1 ));

    // Advertise
    path_pub_ = n_.advertise<nav_msgs::Path>( path_topic_, 5 );
    visu_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_markers", 5 );

    // Init and start action server
    registerGoalCallback( boost::bind( &CombinedPlannerNode::actionGoalCB, this ));
    start();
}

void CombinedPlannerNode::update()
{
    if ( !isActive())
        return;

    if ( isPreemptRequested()) {
        deactivate( PREEMPT, GoToResult::STOP );
        return;
    }

    // Try to get the current position
    Pose2d robot_pose;
    if ( !getRobotPose( robot_pose, gmap_frame_id_ )) {
        deactivate( ABORT, GoToResult::PLANNER_ERROR );
        return;
    }

    // Check if we reached the goal
    if ( planner_.isGoalReached( robot_pose )) {
        ROS_INFO( "Goal reached." );
        deactivate( SUCCEEDED, GoToResult::GOAL_REACHED );
        return;
    }

    // Collision? Wait at least 2 second
    bool force_replan = false;
    if ( collision_ ) {
        if ((ros::Time::now() - collision_stamp_).toSec() < 2.0 )
            return; // Still waiting

        // Alright. Force a new local path
        ROS_INFO( "Trying to find a new local path to resolve collision." );
        force_replan = true;
        collision_ = false;
    }

    // Path lost?
    force_replan = force_replan || path_lost_;
    path_lost_ = false;

    lmap_ros_.getCostmapCopy( lmap_cpy_ );
    planner_.setLocalMap( &lmap_wrapper_ );
    try {
        planner_.update( robot_pose, force_replan );
    } catch ( NoPathException& ex ) {
        ROS_ERROR( "No path. Reason: %s", ex.what());
        deactivate( ABORT, GoToResult::NO_PATH );
        return;
    } catch ( PlannerException& ex ) {
        ROS_ERROR( "Severe planner error: %s", ex.what());
        deactivate( ABORT, GoToResult::PLANNER_ERROR );
        return;
    }

    // Reactivate motion control?
    if ( motion_ac_.getState() == SimpleClientGoalState::ABORTED
         || motion_ac_.getState() == SimpleClientGoalState::PREEMPTED ) {
        ROS_WARN( "Reactivating motion control." );
        activate();
    }

    if ( !isActive())
        return; // There might have been an error during reactivation of motion control

    // Publis new local path and feedback
    /// @todo Implement
    //as_feedback_.base_position = robot_pose;
    //publishFeedback( as_feedback_ );
    if ( !planner_.hasNewLocalPath())
        return;

    // Publish new path and waypoint marker
    activate();
    publishLocalPath( planner_.getLocalPath());
    visualizeWaypoints( planner_.getGlobalWaypoints(), "waypoints", 3 );
}

void CombinedPlannerNode::actionGoalCB()
{
    // Stop motion control etc if necessary
    ROS_INFO( "Received new goal" );

    if ( isActive())
        deactivate( PREEMPT, GoToResult::STOP );

    as_goal_ = *acceptNewGoal();
    collision_ = false;
    path_lost_ = false;
    planner_.reset();
    collision_stamp_ = ros::Time::now() - ros::Duration( 10.0 );
    collision_pose_.x = collision_pose_.y = collision_pose_.theta = 0;

    // Check if there is a pending preempt on the new goal
    if ( isPreemptRequested()) {
        ROS_INFO( "Preempt request on new goal." );
        deactivate( PREEMPT, GoToResult::STOP );
        return;
    }

    // Convert goal int map coordinate system if necessary
    geometry_msgs::PoseStamped goal_map;
    if ( as_goal_.target_pose.header.frame_id.compare( gmap_frame_id_ ) != 0 ) {
        try {
            tf_.transformPose( gmap_frame_id_, as_goal_.target_pose, goal_map );
        } catch (tf::TransformException& ex) {
            ROS_ERROR( "Cannot transform goal into map coordinates. Reason: %s",
                       ex.what());
            deactivate( ABORT, GoToResult::PLANNER_ERROR );
            return;
        }
    } else {
        goal_map = as_goal_.target_pose;
    }

    // Get the current robot pose in map coordinates
    lib_path::Pose2d robot_pose;
    if (!getRobotPose( robot_pose, gmap_frame_id_ )) {
        deactivate( ABORT, GoToResult::PLANNER_ERROR );
        return;
    }

    // Run planner
    lmap_ros_.getCostmapCopy( lmap_cpy_ );
    planner_.setLocalMap( &lmap_wrapper_ );
    lib_path::Pose2d pathStart = robot_pose;
    lib_path::Pose2d pathEnd( goal_map.pose.position.x,
                              goal_map.pose.position.y,
                              tf::getYaw( goal_map.pose.orientation ));
    try  {
        planner_.setGoal( pathStart, pathEnd );
    } catch ( NoPathException& ex ) {
        ROS_ERROR( "No path. Reason: %s", ex.what());
        deactivate( ABORT, GoToResult::NO_PATH );
        return;
    } catch ( PlannerException& ex ) {
        ROS_ERROR( "Severe planner error: %s", ex.what());
        deactivate( ABORT, GoToResult::PLANNER_ERROR );
        return;
    }

    // Send goal to motion control
    activate();

    // Visualize waypoints
    visualizeWaypoints( planner_.getGlobalWaypoints(), "waypoints", 3 );
}

void CombinedPlannerNode::globalMapCB( const nav_msgs::OccupancyGridConstPtr &map )
{
    gmap_cpy_ = *map;
    gmap_frame_id_ = "/map";

    // Ready to go
    got_map_ = true;
    planner_.setGlobalMap( &gmap_wrapper_ );
}

void CombinedPlannerNode::activate()
{
    // Wait for action server
    if ( !motion_ac_.waitForServer( ros::Duration( 1.0 )) ) {
        ROS_WARN( "Motion control action server didn't connect within 1.0 seconds" );
        deactivate( ABORT, GoToResult::MOTION_CTRL_ERROR );
        return;
    }

    // Send goal
    motion_control::MotionGoal motionGoal;
    motionGoal.mode = motion_control::MotionGoal::MOTION_FOLLOW_PATH;
    motionGoal.path_topic = path_topic_;
    plannerPathToRos( planner_.getLocalPath(), motionGoal.path.poses );
    motionGoal.v = as_goal_.max_speed;
    motionGoal.pos_tolerance = 0.20;
    motion_ac_.sendGoal( motionGoal, boost::bind( &CombinedPlannerNode::motionCtrlDoneCB, this, _1, _2 ));
    publishLocalPath( planner_.getLocalPath());
}

void CombinedPlannerNode::deactivate( ActionResultState state, int result )
{
    ROS_INFO( "Deactivating path planner. Action state: %d Result: %d", state, result );
    motion_ac_.cancelAllGoals();
    publishEmptyLocalPath();
    planner_.reset();

    as_result_.result = result;
    switch ( state ) {
    case SUCCEEDED:
        setSucceeded( as_result_ );
        break;
    case ABORT:
        /// @todo Somethin is wrong with this
        setAborted( as_result_ );
        //setPreempted( as_result_ );
        break;
    case PREEMPT:
        setPreempted( as_result_ );
        break;
    }
}

void CombinedPlannerNode::motionCtrlDoneCB( const actionlib::SimpleClientGoalState &state,
                                            const motion_control::MotionResultConstPtr &result )
{
    if ( state == SimpleClientGoalState::PREEMPTED ) {
        // We ignore this since we have requested the preempt
        return;
    }

    lib_path::Pose2d robot_pose;
    switch ( result->status ) {
    case motion_control::MotionResult::MOTION_STATUS_COLLISION:
        ROS_WARN( "Motion control reported a collision." );
        // That's the interesting part. Try to resolve the situation if the
        // latest collision is at least 2 seconds ago or if we moved at least a little bit
        if ( getRobotPose( robot_pose, gmap_frame_id_ ) &&
             ((ros::Time::now() - collision_stamp_).toSec() > 3.0
              || !robot_pose.isEqual( collision_pose_, 0.4, 90.0*M_PI/180.0 ))) {
            collision_ = true;
            collision_stamp_ = ros::Time::now();
            collision_pose_ = robot_pose;
        } else {
            ROS_WARN( "Cannot resume from collision." );
            deactivate( ABORT, GoToResult::COLLISION_ERROR );
        }
        break;

    case motion_control::MotionResult::MOTION_STATUS_PATH_LOST:
        // Replan
        ROS_WARN( "Robot is too far away from path! Forcing a replan." );
        path_lost_ = true;
        break;

    default:
        /// @todo Motion control should be active until we cancel it
        ROS_WARN( "Motion control action is done." );
        break;
    }
}

void CombinedPlannerNode::plannerPathToRos(const list<Pose2d> &planner_p, vector<geometry_msgs::PoseStamped>& ros_p )
{
    std::list<lib_path::Pose2d>::const_iterator it = planner_p.begin();
    geometry_msgs::PoseStamped pose;
    for ( ; it != planner_p.end(); it++ ) {
        pose.pose.position.x = it->x;
        pose.pose.position.y = it->y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw( it->theta );
        ros_p.push_back( pose );
    }
}

void CombinedPlannerNode::publishLocalPath( const std::list<lib_path::Pose2d> &path )
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = gmap_frame_id_;
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
    path_msg.header.frame_id = gmap_frame_id_;
    path_msg.header.stamp = ros::Time::now();
    path_pub_.publish( path_msg );
}

bool CombinedPlannerNode::getRobotPose( lib_path::Pose2d& pose, const std::string& map_frame )
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
    pose.x = msg.transform.translation.x;
    pose.y = msg.transform.translation.y;
    pose.theta = tf::getYaw( msg.transform.rotation );

    return true;
}

void CombinedPlannerNode::visualizePath(
        const std::list<lib_path::Point2d> &path,
        const std::string& ns,
        const int color,
        const int id )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = gmap_frame_id_;
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

void CombinedPlannerNode::visualizeWaypoints( const list<lib_path::Pose2d> &wp, string ns, int id ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = gmap_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = ns;
    marker.color.g = 1.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 0.1f;
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
