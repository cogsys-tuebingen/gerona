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

// project
#include "ExploreFrontier.h"

///////////////////////////////////////////////////////////////////////////////
// class ExplorFrontier
///////////////////////////////////////////////////////////////////////////////


namespace frontier_explore {

ExploreFrontier::ExploreFrontier() :
    min_frontier_length_( 0.6 ),
    path_length_gain_( 1.0 ),
    orientation_change_gain_( 1.0 ),
    frontier_length_gain_( 1.0 ),
    planner_(NULL),
    frontiers_()
{
}

ExploreFrontier::~ExploreFrontier()
{ /* Nothing to do */ }

bool ExploreFrontier::getFrontiers( const CvMap& map, std::vector<geometry_msgs::Pose>& frontiers ) {
    findFrontiers( map );
    if ( frontiers_.size() == 0 )
        return false;

    frontiers.clear();
    for ( std::size_t i = 0; i < frontiers_.size(); ++i )
        frontiers.push_back( frontiers_[i].pose );

    return (frontiers.size() > 0);
}

void ExploreFrontier::findFrontiers( const CvMap& map ) {
    frontiers_.clear();

    int w = map.image->width;
    int h = map.image->height;
    unsigned int size = ( w * h );

    bool visited[size];
    memset( visited, false, size*sizeof(bool));
    std::list<unsigned int> toVisit;
    bool left, right, top, bottom;
    std::vector<FrontierPoint> frontier;
    int minIdx, maxIdx, minRowIdx, maxRowIdx;

    // For all cells
    for ( unsigned int i = 0; i < size; ++i ) {
        if ( !map.isOpen( i ) || visited[i] )
            continue;

        frontier.clear();
        toVisit.push_back( i );
        minIdx = minRowIdx = size;
        maxIdx = maxRowIdx = -1;
        while ( !toVisit.empty()) {
            unsigned int cell = toVisit.back();
            toVisit.pop_back();

            left = right = top = bottom = false;
            left = cell > 0 && (cell % w) != 0;
            right = cell < (size - 1) && (cell + 1) % w != 0;
            top = (int)cell >= w;
            bottom = cell < (size - w);

            unsigned int count = 0.0;
            btVector3 dir(0,0,0);
            if ( left && map.isNoInformation( cell-1 )) {
                count++;
                dir += btVector3(-1, 0, 0);
            }
            if ( right && map.isNoInformation( cell+1 )) {
                ++count;
                dir += btVector3(1, 0, 0);
            }
            if ( top && map.isNoInformation( cell-w )) {
                ++count;
                dir += btVector3(0, -1, 0);
            }
            if ( bottom && map.isNoInformation( cell+w )) {
                ++count;
                dir += btVector3(0, 1, 0);
            }

            visited[cell] = true;
            if ( count == 0 )
                continue;
            frontier.push_back( FrontierPoint( cell, dir / (double)count ));
            minIdx = std::min((int)cell, minIdx );
            maxIdx = std::max((int)cell, maxIdx );
            minRowIdx = std::min((int)cell % w, minRowIdx );
            maxRowIdx = std::max((int)cell % w, maxRowIdx );

            // Current cell is a frontier cell. Visited all open and unvisited neighbor cells
            if ( top && left && !visited[cell-w-1] && map.isOpen( cell-w-1 ))
                toVisit.push_back( cell-w-1 );
            if ( top && !visited[cell-w] && map.isOpen( cell-w ))
                toVisit.push_back( cell-w );
            if ( top && right && !visited[cell-w+1] && map.isOpen( cell-w+1 ))
                toVisit.push_back( cell-w+1 );
            if ( left && !visited[cell-1] && map.isOpen( cell-1 ))
                toVisit.push_back( cell-1 );
            if ( right && !visited[cell+1] && map.isOpen( cell+1 ))
                toVisit.push_back( cell+1 );
            if ( bottom && left && !visited[cell+w-1] && map.isOpen( cell+w-1 ))
                toVisit.push_back( cell+w-1 );
            if ( bottom && !visited[cell+w] && map.isOpen( cell+w ))
                toVisit.push_back( cell+w );
            if ( bottom && right && !visited[cell+w+1] && map.isOpen( cell+w+1 ))
                toVisit.push_back( cell+w+1 );
        }

        // Frontier found?
        if ( frontier.size() == 0 )
            continue;

        // Check frontier length
        if ( std::max( map.getDistance( minIdx, maxIdx ),
                       map.getDistance( minRowIdx, maxRowIdx ))
             < min_frontier_length_ )
            continue;

        // Compute frontier orientation
        btVector3 d( 0, 0, 0 );
        for ( unsigned int j = 0; j < frontier.size(); ++j ) {
            d += frontier[j].d;
        }
        double frontierYaw = btAtan2(d.y(), d.x());
        // Take the middle cell as frontier position
        unsigned int middle = frontier.size() / 2;
        if ( frontier.size() < 2 )
            middle = 0;
        double x, y;
        map.celltoWorld( frontier[middle].idx, x, y );

        // Create and store detected frontier
        Frontier newFrontier;
        newFrontier.pose.position.x = x;
        newFrontier.pose.position.y = y;
        newFrontier.pose.position.z = 0;
        newFrontier.pose.orientation = tf::createQuaternionMsgFromYaw( frontierYaw );
        newFrontier.size = frontier.size();
        frontiers_.push_back( newFrontier );
    }
}

bool ExploreFrontier::getExplorationGoals(
        const CvMap& map,
        geometry_msgs::Pose robot_pose,
        std::vector<geometry_msgs::Pose>& goals )
{
    // Calculate frontiers
    findFrontiers( map );
    ROS_INFO( "Found %d frontiers", (int)frontiers_.size());
    if (frontiers_.size() == 0)
        return false;

    // Robot pose on map?
    unsigned int cell = 0;
    if ( !map.worldToCell( robot_pose.position.x, robot_pose.position.y, cell )) {
        ROS_WARN( "Robot position (%f %f) not on map.", robot_pose.position.x, robot_pose.position.y );
        return false;
    }

    // Initialize planner
    if ( planner_ == NULL ) {
        planner_ = new AStar( map.image, CVMAP_OPEN, 2, 0, false );
    } else {
        planner_->setNewMap( map.image );
    }

    // Compute weighted frontiers
    std::vector<WeightedFrontier> weightedFrontiers;
    WeightedFrontier goal;
    for ( std::size_t i = 0; i < frontiers_.size(); ++i ) {
        goal.frontier = frontiers_[i];
        if ( getFrontierCost( map, goal.frontier, robot_pose, goal.cost ))
            weightedFrontiers.push_back( goal );
    }

    goals.clear();
    goals.reserve( weightedFrontiers.size());
    std::sort( weightedFrontiers.begin(), weightedFrontiers.end());
    for ( std::size_t i = 0; i < weightedFrontiers.size(); ++i ) {
        goals.push_back(weightedFrontiers[i].frontier.pose);
    }
    ROS_INFO( "Found %d possible goals", (int)goals.size());
    return goals.size() > 0;
}

bool ExploreFrontier::getFrontierCost( const CvMap& map,
                                       const Frontier& frontier,
                                       const geometry_msgs::Pose& robot_pose,
                                       double &cost ) {
    double path_length, ori_change;
    if ( planner_ != NULL ) {
        // Plan a path from the current position to the frontier
        waypoint_t start, end;
        map.worldToXY( robot_pose.position.x, robot_pose.position.y, start.x, start.y );
        map.worldToXY( frontier.pose.position.x, frontier.pose.position.y, end.x, end.y );
        path_t* path = planner_->planPath( start, end );

        // Frontier is reachable?
        if ( path->size() == 0 )
            return false;

        // Calculate frontier cost parameters
        path_length = path->size() * map.resolution; // Well, this is an approximation...

        // The difference between the robot orientation and the frontier orientation is
        // important if the robot is close to the frontier. It is not so important
        // if the frontier is far away. In this case we check the difference between the
        // robot orientation and the start of the planned path
        double robot_yaw = tf::getYaw( robot_pose.orientation );;
        if ( path_length < 2.0 || path->size() < 2 ) {
            ori_change = std::fabs( robot_yaw - tf::getYaw( frontier.pose.orientation ));
            while ( ori_change > M_PI )
                ori_change -= 2.0*M_PI;
        } else {
            // First waypoint is the start cell
            btVector3 path_ori((*path)[1].x - start.x, (*path)[1].y - start.y, 0.0 );
            ori_change = std:: fabs( path_ori.angle( btVector3( std::cos( robot_yaw ), std::sin( robot_yaw ), 0.0 )));
        }

        ROS_INFO( "path length: %f ori change: %f", path_length, ori_change );

    } else {
        // TODO Compute something usefull here
        path_length = 1.0;
        ori_change = 1.0;
    }

    // Calculate final frontier cost
    cost = path_length_gain_ * path_length
            + orientation_change_gain_ * ori_change
            - frontier_length_gain_ * map.resolution * frontier.size;

    return true;
}

/*bool ExploreFrontier::getExplorationGoals(Costmap2DROS& costmap, tf::Stamped<tf::Pose> robot_pose, navfn::NavfnROS* planner, std::vector<geometry_msgs::Pose>& goals, double potential_scale, double orientation_scale, double gain_scale)
{
    findFrontiers(costmap);
    if (frontiers_.size() == 0)
        return false;

    geometry_msgs::Point start;
    start.x = robot_pose.getOrigin().x();
    start.y = robot_pose.getOrigin().y();
    start.z = robot_pose.getOrigin().z();

    planner->computePotential(start);

    planner_ = planner;
    costmapResolution_ = costmap.getResolution();

    //we'll make sure that we set goals for the frontier at least the circumscribed
    //radius away from unknown space
    float step = -1.0 * costmapResolution_;
    int c = ceil(costmap.getCircumscribedRadius() / costmapResolution_);
    WeightedFrontier goal;
    std::vector<WeightedFrontier> weightedFrontiers;
    weightedFrontiers.reserve(frontiers_.size() * c);
    for (uint i=0; i < frontiers_.size(); i++) {
        Frontier& frontier = frontiers_[i];
        WeightedFrontier weightedFrontier;
        weightedFrontier.frontier = frontier;

        tf::Point p(frontier.pose.position.x, frontier.pose.position.y, frontier.pose.position.z);
        tf::Quaternion bt;
        tf::quaternionMsgToTF(frontier.pose.orientation, bt);
        tf::Vector3 v(cos(bt.getAngle()), sin(bt.getAngle()), 0.0);

        for (int j=0; j <= c; j++) {
            tf::Vector3 check_point = p + (v * (step * j));
            weightedFrontier.frontier.pose.position.x = check_point.x();
            weightedFrontier.frontier.pose.position.y = check_point.y();
            weightedFrontier.frontier.pose.position.z = check_point.z();

            weightedFrontier.cost = potential_scale * getFrontierCost(weightedFrontier.frontier) + orientation_scale * getOrientationChange(weightedFrontier.frontier, robot_pose) - gain_scale * getFrontierGain(weightedFrontier.frontier, costmapResolution_);
            //      weightedFrontier.cost = getFrontierCost(weightedFrontier.frontier) - getFrontierGain(weightedFrontier.frontier, costmapResolution_);
            //      ROS_DEBUG("cost: %f (%f * %f + %f * %f - %f * %f)",
            //          weightedFrontier.cost,
            //          potential_scale,
            //          getFrontierCost(weightedFrontier.frontier),
            //          orientation_scale,
            //          getOrientationChange(weightedFrontier.frontier, robot_pose),
            //          gain_scale,
            //          getFrontierGain(weightedFrontier.frontier, costmapResolution_) );
            weightedFrontiers.push_back(weightedFrontier);
        }
    }

    goals.clear();
    goals.reserve(weightedFrontiers.size());
    std::sort(weightedFrontiers.begin(), weightedFrontiers.end());
    for (uint i = 0; i < weightedFrontiers.size(); i++) {
        goals.push_back(weightedFrontiers[i].frontier.pose);
    }
    return (goals.size() > 0);
}*/

/*double ExploreFrontier::getOrientationChange( const Frontier& frontier, const geometry_msgs::Pose& robot_pose ) {
    double robot_yaw = tf::getYaw( robot_pose.orientation );
    double robot_atan2 = atan2(robot_pose.position.y + sin(robot_yaw), robot_pose.position.x + cos(robot_yaw));
    double frontier_atan2 = atan2(frontier.pose.position.x, frontier.pose.position.y);
    return robot_atan2 - frontier_atan2;
}*/

} // Namespace frontier_explore
