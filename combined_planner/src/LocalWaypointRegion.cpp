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

LocalWaypointRegion::LocalWaypointRegion( const lib_path::Pose2d &center, const double dist_step )
{
    generateGoals( center, dist_step );
}

bool LocalWaypointRegion::getNextGoal( Pose2d &goal )
{
    if ( goal_list_.empty())
        return false;

    goal = goal_list_.front();
    goal_list_.pop_front();
    return true;
}

void LocalWaypointRegion::generateGoals( const lib_path::Pose2d& center, const double dist_step )
{
    // First is the center pose itself
    goal_list_.push_back( center );

    // Calculate orthogonal step vector
    Point2d side_step( cos( center.theta + 0.5*M_PI), sin( center.theta + 0.5*M_PI ));

    // Add the shifted poses
    Pose2d shifted( center );
    shifted.x += dist_step * side_step.x;
    shifted.y += dist_step * side_step.y;
    goal_list_.push_back( shifted );
    shifted.theta += 0.25;
    goal_list_.push_back( shifted );
    shifted  = center;
    shifted.x -= dist_step * side_step.x;
    shifted.y -= dist_step * side_step.y;
    goal_list_.push_back( shifted );
    shifted.theta -= 0.25;
    goal_list_.push_back( shifted );

    /// @todo Hack!
    /*side_step.x = sin( center.theta ); side_step.y = cos( center.theta );
    shifted = center;
    shifted.x += dist_step * side_step.x;
    shifted.y += dist_step * side_step.y;
    goal_list_.push_back( shifted );
    shifted  = center;
    shifted.x -= dist_step * side_step.x;
    shifted.y -= dist_step * side_step.y;
    goal_list_.push_back( shifted );*/
}

