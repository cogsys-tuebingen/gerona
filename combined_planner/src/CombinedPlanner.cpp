/**
 * @file CombinedPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// C/C++
#include <cmath>

// Project
#include "CombinedPlanner.h"
#include "CombinedPlannerException.h"

using namespace lib_path;
using namespace std;
using namespace combined_planner;

CombinedPlanner::CombinedPlanner()
    : lmap_(NULL),
      gmap_(NULL),
      goal_dist_eps_( 0.25 ),
      goal_angle_eps_( 20.0*M_PI/180.0 ),
      wp_dist_eps_( 1.0 ),
      wp_angle_eps_( 0.6 ),
      local_replan_dist_( 1.5 ),
      local_replan_theta_( 30.0*M_PI/180.0 ),
      valid_path_( false ),
      new_local_path_( false )
{
    gplanner_ = NULL; // We gonna create the global planner on the first global map update
    lplanner_  = new LocalPlanner();
}

CombinedPlanner::~CombinedPlanner()
{
    if ( lplanner_ != NULL )
        delete lplanner_;
    if ( gplanner_ != NULL )
        delete gplanner_;
}


void CombinedPlanner::setGoal( const lib_path::Pose2d& robot_pose, const Pose2d &goal )
{
    valid_path_ = false;

    // Try to find a global path
    if ( !findGlobalPath( robot_pose, goal )) {
        return;
    }

    // Try to find a local path to the first waypoint
    valid_path_ = true;
    lpath_.reset();
    update( robot_pose, true );
}

void CombinedPlanner::update( const Pose2d &robot_pose, bool force_replan )
{
    new_local_path_ = false;

    // Reached the goal or we don't have a path?
    if ( isGoalReached( robot_pose ) || !valid_path_ ) {
        valid_path_ = false;
        return;
    }
    valid_path_ = true; // Set this to false only if there is an error

    // Do we have a local and a global map?
    if ( lmap_ == NULL || gmap_ == NULL )
        throw CombinedPlannerException( "Got no local map or no global map on planner update." );

    // Update the current local path. This will remove allready reached local waypoints
    lpath_.updateWaypoints( 0.25, robot_pose );

    // debug
    if ( !lpath_.areWaypointsFree( lmap_->getResolution(), lmap_ ))
        ROS_WARN( "Local path is not free." );

    // Replan anyway?
    force_replan = force_replan || lpath_.getWaypointCount() <= 0 ||
            (!gwaypoints_.empty() && robot_pose.isEqual( lpath_.getEnd(), wp_dist_eps_, wp_angle_eps_ )) ||
            !lpath_.areWaypointsFree( lmap_->getResolution(), lmap_ );

    // Something to do?
    if ( !force_replan )
        return;

    // Plan a new local path
    ROS_INFO( "Planning a new local path." );
    if ( lplanner_->planPath( robot_pose, gwaypoints_, ggoal_ )) {
        lpath_ = lplanner_->getPath();
        new_local_path_ = true;
        ROS_INFO( "Found new local path." );
        return;
    }

    /* We didn't find a local path. Search a new global path if we got a new global
     * of if we changed our position since the last global replan. */
    /// @todo Allow global replan if there is a new global map
    if ( !gstart_.isEqual( robot_pose, 0.5, 0.5 )) {
        ROS_WARN( "Searching a new global path because there is no local one." );
        if ( findGlobalPath( robot_pose, ggoal_ )) {
            // Found a new global path. Update the local path
            lpath_.reset();
            update( robot_pose, true );
            return;
        }
    }

    // Fail
    valid_path_ = false;
    ROS_ERROR( "No local path found." );
}

bool CombinedPlanner::findGlobalPath( const Pose2d &start, const Pose2d &goal )
{
    ROS_INFO( "Planing a global path." );

    // Check if we got a global map/global planner was initialized
    if ( gmap_ == NULL || gplanner_ == NULL ) {
        throw CombinedPlannerException( "No global map." );
    }

    // Plan path
    if ( gplanner_->planPath( Point2d( start.x, start.y ), Point2d( goal.x, goal.y ))) {
        // Calculate waypoints on success
        calculateWaypoints( gplanner_->getLatestPath(), goal, gwaypoints_ );
        // Remeber start/end
        gstart_ = start;
        ggoal_ = goal;
        return true;
    }
    ROS_WARN( "No global path found" );
    return false;
}


bool CombinedPlanner::isGoalReached( const Pose2d& robot_pose ) const
{
    if ( !gwaypoints_.empty())
        return false; // Too much waypoints left

    // Check distance to the global goal
    double d_dist, d_theta;
    getPoseDelta( ggoal_, robot_pose, d_dist, d_theta );
    return (d_dist < goal_dist_eps_ && d_theta < goal_angle_eps_);
}

void CombinedPlanner::calculateWaypoints( const vector<Point2d> &path, const Pose2d& goal, list<Pose2d> &waypoints ) const
{
    double min_waypoint_dist_ = 0.75;
    double max_waypoint_dist_ = 1.0;

    waypoints.clear();

    // Path too short? We need at least a start and an end point
    if ( path.size() < 2 ) {
        //waypoints.push_back( goal );
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
                //waypoints.push_back( goal );
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

lib_path::Pose2d CombinedPlanner::getNormalizedDelta( const Point2d &start, const Point2d &end ) const
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

void CombinedPlanner::getPoseDelta( const Pose2d &p, const Pose2d &q,
                                    double &d_dist, double &d_theta ) const
{
    d_dist = sqrt( pow( p.x - q.x, 2 ) + pow( p.y - q.y, 2 ));
    d_theta = fabs( p.theta - q.theta );
    while ( d_theta > M_PI ) /// @todo use lib utils
        d_theta -= 2.0*M_PI;
    d_theta = fabs( d_theta );
}

void CombinedPlanner::setGlobalMap( GridMap2d *gmap )
{
    gmap_ = gmap;
    if ( gplanner_ == NULL )
        gplanner_ = new GlobalPlanner( gmap_ );
    else
        gplanner_->setMap( gmap_ );
}

void CombinedPlanner::setLocalMap( GridMap2d *lmap )
{
    lmap_ = lmap;
    lplanner_->setMap( lmap_ );
}

