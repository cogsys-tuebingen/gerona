/**
 * @file GlobalPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <ros/ros.h>

// Project
#include "GlobalPlanner.h"
#include "PlannerExceptions.h"

using namespace lib_path;
using namespace combined_planner;

GlobalPlanner::GlobalPlanner( lib_path::GridMap2d *map )
    : map_( map ),
      a_star_( map ),
      robot_bubble_( 0.35 )
{ /* Nothing else to do */ }


void GlobalPlanner::setMap( GridMap2d *map )
{
    map_ = map;
    a_star_.setNewMap( map );
}

void GlobalPlanner::planPath( Point2d start, Point2d goal )
{
    // Clean old paths
    path_.clear();
    path_raw_.clear();

    // Check start and goal pose
    if ( !map_->isInMap( start ) || !map_->isInMap( goal ))
        throw PlannerException( "Start or goal pose lies outside of the map." );


    // Buffer map data at current position
    CircleBuffer start_area( start, robot_bubble_, map_ );
    map_->getAreaValues( start_area );

    // Clear current position
    map_->setAreaValue( start_area, 0 );

    // Convert start/goal into cell coordinates
    waypoint_t startWayp, goalWayp;
    unsigned int x, y;
    map_->point2cell( start, x, y );
    startWayp.x = x; startWayp.y = y;
    map_->point2cell( goal, x, y );
    goalWayp.x = x; goalWayp.y = y;

    // Run A*
    bool path_found = a_star_.planPath( startWayp, goalWayp );
    /// @todo Fix this!
    //map_->setAreaValue( start_area );

    if ( !path_found )
        throw NoPathException( "No global path found." );

    // Convert the raw path to map coordinates
    path_t* cell_path = a_star_.getLastPath();
    Point2d p;
    for ( std::size_t i = 0; i < cell_path->size(); ++i ) {
        map_->cell2point((*cell_path)[i].x, (*cell_path)[i].y, p );
        path_raw_.push_back( p );
    }


    // Path too short? Flattening not necessary!
    if ( cell_path->size() <= 2 ) {
        path_.assign( path_raw_.begin(), path_raw_.end());
        return;
    }

    // Flatten path
    path_.push_back( start );
    flatten( cell_path, path_ );
    path_.push_back( goal );

    return;
}

void GlobalPlanner::flatten( const path_t* raw, std::vector<Point2d> &flattened ) const
{
    Point2d p;
    std::size_t first = 0;
    std::size_t current = 1;
    const std::size_t look_ahead = 25; // Always check next n points
    while ( current < raw->size() - 1 ) {
        if ( isLineFree((*raw)[first], (*raw)[current])) {
            current++;
        } else {
            bool succ = false;
            for ( size_t i = 1; (i + current) < (raw->size() - 1) && i < look_ahead; ++i ) {
                if ( isLineFree((*raw)[first], (*raw)[current + i])) {
                    current += i;
                    succ = true;
                    break;
                }
            }

            if ( !succ ) {
                /// @todo Think about the -1 here
                map_->cell2point((*raw)[current-1].x, (*raw)[current-1].y, p );
                flattened.push_back( p );
                first = current - 1;
                current++;
            }
        }
    }
}

bool GlobalPlanner::isLineFree( const lib_path::waypoint_t &p1,
                                const lib_path::waypoint_t &p2 ) const {
    int x0, y0, x1, y1;
    x0 = p1.x; y0 = p1.y;
    x1 = p2.x; y1 = p2.y;

    // Taken from http://de.wikipedia.org/wiki/Bresenham-Algorithmus
    int dx =  abs( x1 - x0 ), sx = x0 < x1 ? 1 : -1;
    int dy = -abs( y1 - y0 ), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    for (;;) {
        if ( !map_->isFree( x0, y0 ))
            return false;

        if ( x0 == x1 && y0 == y1 )
            return true;

        e2 = 2*err;
        if (e2 > dy) { err += dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}
