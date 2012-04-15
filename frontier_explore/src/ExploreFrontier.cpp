/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*
 * File: ExploreFrontier.cpp
 * Created: Jan 2012
 *
 * The original code was taken from ROS package "exploration/explore".
 *
 * Original author: Duhadway
 * Modified by: Marks
 *
 */

// C/C++
#include <cmath>

// Workspace
#include <utils/LibUtil/MathHelper.h>

// Project
#include "ExploreFrontier.h"

///////////////////////////////////////////////////////////////////////////////
// class ExplorFrontier
///////////////////////////////////////////////////////////////////////////////


namespace frontier_explore {

ExploreFrontier::ExploreFrontier() :
    frontiers_(),
    min_frontier_length_( 0.6 ),
    path_length_gain_( 1.0 ),
    orientation_change_gain_( 1.0 ),
    frontier_length_gain_( 1.0 ),
    planner_( NULL )
{
}

ExploreFrontier::~ExploreFrontier()
{
    /* Nothing to do */
}

bool ExploreFrontier::getFrontiers( const lib_path::GridMap2d &map, std::vector<Frontier> &frontiers ) {
    findFrontiers( map );

    if ( frontiers_.size() == 0 )
        return false;

    frontiers = frontiers_;
    return frontiers.size() > 0;
}

void ExploreFrontier::findFrontiers( const lib_path::GridMap2d &map ) {
    frontiers_.clear();

    int w = map.getWidth();
    int h = map.getHeight();
    unsigned int size = ( w * h );

    if ( size <= 0 )
        return;

    bool visited[size];
    memset( visited, false, size*sizeof(bool));
    std::list<unsigned int> to_visit;
    bool left, right, top, bottom;
    std::vector<FrontierPoint> frontier;
    int min_x, max_x, min_y, max_y;
    unsigned int cell;

    // For all cells
    for ( unsigned int y = 0; (int)y < h; ++y ) {
        for ( unsigned int x = 0; (int)x < w; ++x ) {
            cell = y*w + x;

            // Occupied or visited?
            if ( !map.isFree( x, y ) || visited[cell] )
                continue;

            // Cell is free and unvisited. Search frontier
            frontier.clear();
            to_visit.push_back( cell );
            min_x = min_y = size;
            max_x = max_y = -1;
            while ( !to_visit.empty()) {
                // Get current cell coordinates
                cell = to_visit.back();
                to_visit.pop_back();
                unsigned int q = cell/w;
                unsigned int p = cell - w*q;

                // Neighbor flags
                left = right = top = bottom = false;
                left = p > 0;
                right = (int)p < (w - 1);
                top = (int)q < (h - 1);
                bottom = q > 0;

                // Check neighbors and set cell visited
                unsigned int count = 0;
                Eigen::Vector3d dir( 0, 0, 0 );
                if ( left && map.isNoInformation( p - 1, q )) {
                    ++count;
                    dir.x() += -1.0;
                }
                if ( right && map.isNoInformation( p + 1, q )) {
                    ++count;
                    dir.x() += 1.0;
                }
                if ( top && map.isNoInformation( p, q + 1 )) {
                    ++count;
                    dir.y() += 1.0;
                }
                if ( bottom && map.isNoInformation( p, q - 1 )) {
                    ++count;
                    dir.y() += -1.0;
                }
                visited[cell] = true;

                // No neighbor with no information found? Cell is not part of a frontier
                if ( count == 0 )
                    continue;

                // Add point to frontier
                frontier.push_back( FrontierPoint( p, q, dir / (double)count ));
                min_x = std::min((int)p, min_x );
                max_x = std::max((int)p, max_x );
                min_y = std::min((int)q, min_y );
                max_y = std::max((int)q, max_y );

                // Visited all open and unvisited neighbor cells
                if ( top && left && !visited[cell+w-1] && map.isFree( p-1, q+1 ))
                    to_visit.push_back( cell+w-1 );
                if ( top && !visited[cell+w] && map.isFree( p, q+1 ))
                    to_visit.push_back( cell+w );
                if ( top && right && !visited[cell+w+1] && map.isFree( p+1, q+1 ))
                    to_visit.push_back( cell+w+1 );
                if ( left && !visited[cell-1] && map.isFree( p-1, q ))
                    to_visit.push_back( cell-1 );
                if ( right && !visited[cell+1] && map.isFree( p +1, q ))
                    to_visit.push_back( cell+1 );
                if ( bottom && left && !visited[cell-w-1] && map.isFree( p-1, q-1 ))
                    to_visit.push_back( cell-w-1 );
                if ( bottom && !visited[cell-w] && map.isFree( p, q-1 ))
                    to_visit.push_back( cell-w );
                if ( bottom && right && !visited[cell-w+1] && map.isFree( p+1, q-1 ))
                    to_visit.push_back( cell-w+1 );
            }

            // Frontier found?
            if ( frontier.size() == 0 )
                continue;

            // Check frontier length
            if ( map.getResolution()*(double)std::max( max_x - min_x, max_y - min_y )
                 < min_frontier_length_ )
                continue;

            // Compute frontier orientation
            Eigen::Vector3d dir_sum( 0, 0, 0 );
            for ( unsigned int j = 0; j < frontier.size(); ++j )
                dir_sum += frontier[j].dir;
            double frontier_yaw = 0;
            if ( dir_sum.norm() > 1E-6 )
                frontier_yaw = atan2(dir_sum.y(), dir_sum.x());

            // Take the middle cell as frontier position
            unsigned int middle = frontier.size() / 2;
            if ( frontier.size() < 2 )
                middle = 0;
            lib_path::Point2d frontier_pos;
            map.cell2point( frontier[middle].x, frontier[middle].y, frontier_pos );

            // Create and store detected frontier
            Frontier newFrontier;
            newFrontier.pose.x() = frontier_pos.x;
            newFrontier.pose.y() = frontier_pos.y;
            newFrontier.pose(2) = frontier_yaw;
            newFrontier.size = frontier.size();
            frontiers_.push_back( newFrontier );
        }
    }
}

bool ExploreFrontier::getExplorationGoals(
        lib_path::GridMap2d& explore_map,
        lib_path::GridMap2d &ground_map,
        const Eigen::Vector3d &robot_pose,
        std::vector<WeightedFrontier> &goals )
{
    // Calculate frontiers
    findFrontiers( explore_map );
    ROS_INFO( "Found %d possible goals", (int)frontiers_.size());
    if ( frontiers_.size() == 0 )
        return false;

    // Robot pose on map?
    if ( !ground_map.isInMap( robot_pose.x(), robot_pose.y()))
        return false;

    // Initialize planner and map
    if ( planner_ == NULL ) {
        planner_ = new lib_path::AStar( &ground_map );
    } else {
        planner_->setNewMap( &ground_map );
    }
    lib_path::CircleArea robot_bubble( lib_path::Point2d( robot_pose.x(), robot_pose.y()), 0.35, &ground_map );
    ground_map.setAreaValue( robot_bubble, 0 );

    // Compute weighted frontiers
    std::vector<WeightedFrontier> weightedFrontiers;
    WeightedFrontier goal;
    for ( std::size_t i = 0; i < frontiers_.size(); ++i ) {
        goal.frontier = frontiers_[i];
        if ( getFrontierCost( ground_map, goal.frontier, robot_pose, goal.cost ))
            weightedFrontiers.push_back( goal );
    }

    std::sort( weightedFrontiers.begin(), weightedFrontiers.end());
    goals.assign( weightedFrontiers.begin(), weightedFrontiers.end());

    ROS_INFO( "Found %d reachable goals", (int)goals.size());
    return goals.size() > 0;
}

bool ExploreFrontier::getFrontierCost( const lib_path::GridMap2d &map,
                                       const Frontier& frontier,
                                       const Eigen::Vector3d &robot_pose,
                                       double &cost ) {
    double path_length, ori_change;
    if ( planner_ != NULL ) {
        // Plan a path from the current position to the frontier
        lib_path::waypoint_t start, end;
        unsigned int x, y;
        if ( !map.point2cell( robot_pose.x(), robot_pose.y(), x, y ))
            return false;
        start.x = x; start.y = y;
        if ( !map.point2cell( frontier.pose.x(), frontier.pose.y(), x, y ))
            return false;
        end.x = x; end.y = y;
        bool path_exists = planner_->planPath( start, end );

        // Frontier is reachable?
        if ( !path_exists )
            return false;

        // Calculate frontier cost parameters
        lib_path::path_t *path = planner_->getLastPath();
        path_length = path->size()*map.getResolution(); // Well, this is an approximation...

        // The difference between the robot orientation and the frontier orientation is
        // important if the robot is close to the frontier. It is not so important
        // if the frontier is far away. In this case we check the difference between the
        // robot orientation and the start of the planned path
        double robot_yaw = robot_pose(2);
        if ( path_length < 2.0 || path->size() < 2 ) {
            ori_change = fabs( MathHelper::AngleDelta( robot_yaw, frontier.pose(2)));
        } else {
            // First waypoint is the start cell
            Eigen::Vector2d path_begin((*path)[1].x - start.x, (*path)[1].y - start.y );
            ori_change = std::fabs( MathHelper::Angle( path_begin, Eigen::Vector2d( cos( robot_yaw ), sin( robot_yaw ))));
        }

    } else {
        /// @todo Compute something usefull here
        path_length = 1.0;
        ori_change = 1.0;
    }

    // Calculate final frontier cost
    cost = path_length_gain_ * path_length
            + orientation_change_gain_ * ori_change
            - frontier_length_gain_ * map.getResolution() * frontier.size;

    return true;
}

} // Namespace frontier_explore
