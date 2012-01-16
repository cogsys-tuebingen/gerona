/**
 * @file GlobalPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <ros/ros.h>

// Project
#include "GlobalPlanner.h"

using namespace lib_path;

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

bool GlobalPlanner::planPath( const lib_path::Point2d &start, lib_path::Point2d &goal )
{
    // Clean old paths
    path_.clear();
    path_raw_.clear();

    // Convert start/goal into cell coordinate
    if ( !map_->isInMap( start ) || !map_->isInMap( goal )) {
        ROS_WARN( "Start or goal lies outside of the map. Path planning not possible!" );
        return false;
    }
    waypoint_t startWayp, goalWayp;
    map_->point2Cell( start, startWayp.x, startWayp.y );
    map_->point2Cell( goal, goalWayp.x, goalWayp.y );

    // Run A*
    if ( !a_star_.planPath( startWayp, goalWayp )) {
        ROS_WARN( "No global path found." );
        return false;
    }

    // Copy the raw path
    path_t* cell_path = a_star_.getLastPath();
    path_raw_.resize( cell_path->size());
    for ( std::size_t i = 0; i < cell_path->size(); ++i )
        map_->cell2point((*cell_path)[i].x, (*cell_path)[i].y, path_raw_[i] );


    // Path too short? Flattening not necessary!
    if ( cell_path->size() <= 2 ) {
        path_.assign( path_raw_.begin(), path_raw_.end());
        return true;
    }

    // Flatten path
    Point2d p;
    std::size_t first = 0;
    std::size_t current = 1;
    std::size_t look_ahead = 0;
    path_.push_back( start );
    while ( current < cell_path->size() - 1 ) {
        if ( isLineFree((*cell_path)[first], (*cell_path)[current])) {
            current++;
        } else {
            // Look ahead hack
            // TODO This is still buggy
            bool succ = false;
            for ( size_t i = 1; (i + current) < (cell_path->size() - 1) && i < 4; ++i ) {
                if ( isLineFree((*cell_path)[first], (*cell_path)[current + i])) {
                    current += i;
                    succ = true;
                    break;
                }
            }

            if ( !succ ) {
                map_->cell2point((*cell_path)[current].x, (*cell_path)[current].y, p );
                path_.push_back( p );
                first = current;
                current++;
            }
        }
    }
    path_.push_back( goal );

    return true;
}

bool GlobalPlanner::isLineFree( const lib_path::waypoint_t &p1,
                                const lib_path::waypoint_t &p2 ) const {
    /*
     * Copied from sickday planner. Authors: dube, laible
     */

    bool free = true;

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
    return free;
}

void GlobalPlanner::getLatestPath( std::list<lib_path::Point2d> &path ) const
{
    path.assign( path_.begin(), path_.end());
}

void GlobalPlanner::getLatestPathRaw( std::list<lib_path::Point2d> &path ) const
{
    path.assign( path_raw_.begin(), path_raw_.end());
}
