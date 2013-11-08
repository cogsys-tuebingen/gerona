/**
 * @file LocalWaypointRegion.cpp
 * @date Jan 2012
 * @author marks
 */

// C/C++
#include <cmath>

// Workspace
#include <utils/LibPath/common/Point2d.h>

// Project
#include "LocalWaypointRegion.h"

using namespace std;
using namespace lib_path;

LocalWaypointRegion::LocalWaypointRegion( const lib_path::Pose2d &center,
                                          const double dist_step,
                                          const double angle_step_deg,
                                          const bool inverse )
{
    generateGoals( center, dist_step, angle_step_deg, inverse );
}

bool LocalWaypointRegion::getNextGoal( Pose2d &goal, double& gain )
{
    if ( goal_list_.empty())
        return false;

    goal = goal_list_.front().first;
    gain = goal_list_.front().second;
    goal_list_.pop_front();
    return true;
}

void LocalWaypointRegion::generateGoals( const lib_path::Pose2d &center,
                                         const double dist_step,
                                         const double angle_steg_deg,
                                         const bool inverse )
{
    double angle_step_rad = M_PI*angle_steg_deg/180.0;

    // First is the center pose itself
    goal_list_.push_back( pair<Pose2d, double>( center, 1.0 ));

    // Calculate orthogonal step vector
    Point2d side_step( cos( center.theta + 0.5*M_PI), sin( center.theta + 0.5*M_PI ));

    // Add the poses shifted by one step
    Pose2d shifted( center );
    shifted.x += dist_step * side_step.x;
    shifted.y += dist_step * side_step.y;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.0 ));
    shifted.theta += angle_step_rad;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.1 ));

    shifted  = center;
    shifted.x -= dist_step * side_step.x;
    shifted.y -= dist_step * side_step.y;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.0 ));
    shifted.theta -= angle_step_rad;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.1 ));

    // Add the poses shifted by two steps
    shifted = center;
    shifted.x += 2.0 * dist_step * side_step.x;
    shifted.y += 2.0 * dist_step * side_step.y;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.2 ));
    shifted.theta += angle_step_rad;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.2 ));

    shifted  = center;
    shifted.x -= 2.0 * dist_step * side_step.x;
    shifted.y -= 2.0 * dist_step * side_step.y;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.2 ));
    shifted.theta -= angle_step_rad;
    goal_list_.push_back( pair<Pose2d, double>( shifted, 1.2 ));

    if ( inverse ) {
        // Allow to reach the pose backwards
        shifted = center;
        shifted.theta += M_PI;
        goal_list_.push_back( pair<Pose2d, double>( shifted, 1.7 ) );
    }
}

