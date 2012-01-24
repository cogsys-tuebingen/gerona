/**
 * @file GlobalPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <ros/ros.h>

// Project
#include "GlobalPlanner.h"
#include "CombinedPlannerException.h"

using namespace lib_path;
using namespace combined_planner;

GlobalPlanner::GlobalPlanner( lib_path::GridMap2d *map )
    : map_( map ),
      a_star_( map ),
      start_( 0, 0 ),
      goal_( 0, 0 )
{ /* Nothing else to do */ }


void GlobalPlanner::setMap( GridMap2d *map )
{
    map_ = map;
    a_star_.setNewMap( map );
}

bool GlobalPlanner::planPath( Point2d start, Point2d goal )
{
    // Clean old paths
    path_.clear();
    path_raw_.clear();

    // Check start and goal pose
    if ( !map_->isInMap( start ) || !map_->isInMap( goal )) {
        throw CombinedPlannerException( "Start or goal pose lies outside of the map." );
    }

    // Convert start/goal into cell coordinate
    waypoint_t startWayp, goalWayp;
    unsigned int x, y;
    map_->point2cell( start, x, y );
    startWayp.x = x; startWayp.y = y;
    map_->point2cell( goal, x, y );
    goalWayp.x = x; goalWayp.y = y;

    // Run A*
    if ( !a_star_.planPath( startWayp, goalWayp )) {
        return false;
    }

    // Copy the raw path
    path_t* cell_path = a_star_.getLastPath();
    Point2d p;
    for ( std::size_t i = 0; i < cell_path->size(); ++i ) {
        map_->cell2point((*cell_path)[i].x, (*cell_path)[i].y, p );
        path_raw_.push_back( p );
    }


    // Path too short? Flattening not necessary!
    if ( cell_path->size() <= 2 ) {
        path_.assign( path_raw_.begin(), path_raw_.end());
        return true;
    }

    // Flatten path
    path_.push_back( start );
    flatten( cell_path, path_ );
    path_.push_back( goal );

    return true;
}

void GlobalPlanner::flatten( const path_t* raw, std::vector<Point2d> &flattened ) const
{
    Point2d p;
    std::size_t first = 0;
    std::size_t current = 1;
    std::size_t look_ahead = 5;
    while ( current < raw->size() - 1 ) {
        if ( isLineFree((*raw)[first], (*raw)[current])) {
            current++;
        } else {
            // Look ahead hack
            /// @todo This is still buggy
            bool succ = false;
            for ( size_t i = 1; (i + current) < (raw->size() - 1) && i < look_ahead; ++i ) {
                if ( isLineFree((*raw)[first], (*raw)[current + i])) {
                    current += i;
                    succ = true;
                    break;
                }
            }

            if ( !succ ) {
                map_->cell2point((*raw)[current-1].x, (*raw)[current-1].y, p );
                flattened.push_back( p );
                first = current;
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

    double dx, dy, err;
    dx = abs( x1 - x0 );
    dy = abs( y1 - y0 );
    err = dx - dy;

    int sx, sy;
    sx = sy = 1;
    if ( x0 >= x1 )
        sx = -1;
    if ( y0 >= y1 )
        sy = -1;

    while ( true ) {
        if ( !map_->isFree( x0, y0 ))
            return false;

        if ( x0 == x1 && y0 == y1 )
            return true;

        if ( 2.0 * err > -dy ) {
            err = err - dy;
            x0 += sx;
        }

        if ( 2.0 * err < dx ) {
            err = err + dx;
            y0 += sy;
        }
    }

    /*
     * Copied from sickday planner. Authors: dube, laible
     */

    /*bool free = true;

    if ( p1.x != p2.x || p1.y != p2.y) {
        // Traverse from left to right
        int x1, y1, x2, y2;
        if (p1.x <= p2.x) {
            x1 = p1.x;
            y1 = p1.y;
            x2 = p2.x;
            y2 = p2.y;
        } else {
            x1 = p2.x;
            y1 = p2.y;
            x2 = p1.x;
            y2 = p1.y;
        }

        int stepX = 1;
        int stepY = (y1 <= y2 ? 1 : -1);

        double tDeltaX = 0.0;
        double tDeltaY = 0.0;

        if (x1 == x2) {
            tDeltaX = 0.0;
            tDeltaY = 1.0;
        } else if (y1 == y2) {
            tDeltaX = 1.0;
            tDeltaY = 0.0;
        } else {
            double m = double(y2 - y1) / double(x2 - x1);
            tDeltaX = sqrt(1.0 + m * m);
            tDeltaY = sqrt(1.0 + 1.0 / (m * m));
        }

        double tMaxX = tDeltaX * 0.5;
        double tMaxY = tDeltaY * 0.5;

        if ( map_ != NULL ) {
            if ( map_->getWidth() > x2 && map_->getHeight() > max(y1, y2)) {
                int x = x1;
                int y = y1;

                while (x <= x2 && (stepY == 1 ? y <= y2 : y >= y2)) {
                    if ( !map_->isFree( x, y )) {
                        free = false;
                        break;
                    }

                    if (tMaxX < tMaxY) {
                        tMaxX += tDeltaX;
                        x += stepX;
                    } else {
                        tMaxY += tDeltaY;
                        y += stepY;
                    }
                }
            } else {
                free = false;
            }
        } else {
            free = false;
        }
    }
    return free;*/
}
