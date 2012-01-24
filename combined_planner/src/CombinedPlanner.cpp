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
      goal_dist_eps_( 1.0 ),
      goal_angle_eps_( 0.6 ),
      wp_dist_eps_( 1.0 ),
      wp_angle_eps_( 0.6 ),
      ggoal_reached_( true )
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


bool CombinedPlanner::setGoal( const Pose2d &robot_pose, const Pose2d &goal )
{
    // Reset
    gwaypoints_.clear();
    ggoal_reached_ = true;

    // Try to find a local path
    if ( !findGlobalPath( robot_pose, goal )) {
        return false;
    }

    // Set new goal
    ggoal_ = goal;
    ggoal_reached_ = false;
    if ( isGoalReached( robot_pose )) {
        setGoalReached();
        return true;
    }

    latest_lgoal_ = goal;

    // Try to find a local path to the first waypoint
    return updateLocalPath( robot_pose, true );
}

bool CombinedPlanner::findGlobalPath( const Pose2d &start, const Pose2d &goal )
{
    // Check if we got a global map/global planner was initialized
    if ( gmap_ == NULL || gplanner_ == NULL ) {
        throw CombinedPlannerException( "No global map." );
    }

    // Plan path
    if ( gplanner_->planPath( Point2d( start.x, start.y ), Point2d( goal.x, goal.y ))) {
        // Calculate waypoints on success
        calculateWaypoints( gplanner_->getLatestPath(), goal, gwaypoints_ );
        ROS_INFO( "Found a global path." );
        return true;
    }
    return false;
}

bool CombinedPlanner::updateLocalPath( const Pose2d &robot_pose, bool force_replan )
{
    // Check if we have reached the goal
    if ( isGoalReached( robot_pose )) {
        setGoalReached();
        return false; // No new local path
    }

    // Select next waypoint?
    Pose2d current_wp;
    bool new_wp = false;
    if ( gwaypoints_.empty()) {
        // Next waypoint must be the goal
        current_wp = ggoal_;
    } else if ( isWaypointReached( robot_pose, latest_lgoal_ )) {
        new_wp = true;
        gwaypoints_.pop_front();
        current_wp = gwaypoints_.front();
    } else {
        current_wp = gwaypoints_.front();
    }

    // Check if we have to replan
    if ( !(new_wp || force_replan ))
        return false;

    // Plan a local path.
    try {
        if ( lplanner_->planPath( robot_pose, current_wp )) {
            // Found a path. Remeber local goal
            latest_lgoal_ = lplanner_->getPath().back();
            return true;
        }
    } catch ( CombinedPlannerException& ex ) {}

    /* Well, we didn't find a local path. Try to invert the orientation of
     * the current waypoint if it's not the global  goal. Maybe there is
     * not enough space to turn around. */
    Pose2d wp_backw( current_wp );
    wp_backw.theta += M_PI;
    try {
        if ( lplanner_->planPath( robot_pose, wp_backw )) {
            // Well, we found a path. Thats the most important thing. Remeber local goal
            latest_lgoal_ = lplanner_->getPath().back();
            return true;
        }
    } catch ( CombinedPlannerException& ex ) {}

    /* Last try. If there is moren than one waypoint left, try to reach that one. */
    if ( gwaypoints_.size() >= 2 ) {
        list<Pose2d>::iterator it = gwaypoints_.begin();
        it++;
        if ( lplanner_->planPath( robot_pose, *it )) {
            gwaypoints_.pop_front();
            latest_lgoal_ = lplanner_->getPath().back();
            return true;
        }
    }

    // Well, there might be a local path but we didn't find one
    throw CombinedPlannerException( "No local path found." );
}

bool CombinedPlanner::isWaypointReached( const Pose2d &robot_pose, const Pose2d &wp ) const
{
    double d_dist, d_theta;
    getPoseDelta( robot_pose, wp, d_dist, d_theta );
    return d_dist < wp_dist_eps_ && d_theta < wp_angle_eps_;
}

bool CombinedPlanner::isGoalReached( const Pose2d &robot_pose ) const {
    if ( ggoal_reached_ )
        return true; // We reached it earlier

    if ( !gwaypoints_.size() > 1)
        return false; // Too much waypoints left

    // Check distance to the latest global goal
    double d_dist, d_theta;
    getPoseDelta( ggoal_, robot_pose, d_dist, d_theta );
    return (d_dist < goal_dist_eps_ && d_theta < goal_angle_eps_);
}

void CombinedPlanner::setGoalReached() {
    gwaypoints_.clear();
    ggoal_reached_ = true;
}

void CombinedPlanner::calculateWaypoints( const vector<Point2d> &path, const Pose2d& goal, list<Pose2d> &waypoints ) const
{
    double min_waypoint_dist_ = 1.0;
    double max_waypoint_dist_ = 1.50;

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
    waypoints.push_back( goal );
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

