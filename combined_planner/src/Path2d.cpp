/**
 * @file Path2d.h
 * @date Feb 2012
 * @author marks
 */

// C/C++
#include <cmath>

// Project
#include "Path2d.h"

using namespace lib_path;
using namespace std;
using namespace combined_planner;

Path2d::Path2d()
{
    reset();
}

Path2d::Path2d( const WaypointList &wp )
{
    setWaypoints( wp );
}

void Path2d::setWaypoints( const WaypointList &wp )
{
    start_ = wp.front();
    end_ = wp.back();
    waypoints_.assign( wp.begin(), wp.end());
}

void Path2d::updateWaypoints( const double radius, const Pose2d &robot_pose )
{
    Pose2d wp;
    double dist_to_wp;
    while ( !waypoints_.empty()) {
        wp = waypoints_.front();
        dist_to_wp = sqrt( pow( wp.x - robot_pose.x, 2 ) + pow( wp.y - robot_pose.y, 2 ));
        if ( dist_to_wp <= radius ) {
            waypoints_.pop_front();
        } else {
            break;
        }
    }
}

bool Path2d::isEndReached( const Pose2d& robot_pose,
                           const double max_d_dist,
                           const double max_d_theta ) const
{
    return end_.isEqual( robot_pose, max_d_dist, max_d_theta );
}

bool Path2d::isFree( double radius, const GridMap2d *map ) const
{
    if ( waypoints_.empty())
        return true;

    WaypointList::const_iterator it = waypoints_.begin();
    CircleArea check_area( *it, radius, map );
    while ( it != waypoints_.end()) {
        check_area.setCenter( *it, map );
        if ( !map->isAreaFree( check_area ))
            return false;
        it++;
    }
    return true;
}

void Path2d::reset()
{
    waypoints_.clear();
    start_.x = start_.y = start_.theta = 0.0;
    end_.x = end_.y = end_.theta = 0.0;
}

void Path2d::addWaypoint( const Pose2d &wp )
{
    if ( waypoints_.empty())
        start_ = wp;
    waypoints_.push_back( wp );
    end_ = wp;
}
