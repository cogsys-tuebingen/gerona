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
 *  Created on: Apr 6, 2009
 *      Author: duhadway
 */

#include <cmath>

#include "ExploreFrontier.h"


using namespace visualization_msgs;

namespace frontier_explore {

ExploreFrontier::ExploreFrontier() :
    min_frontier_length_( 0.6 ),
    planner_(NULL),
    lastMarkerCount_(0),
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

bool ExploreFrontier::getFrontierCost( const CvMap& map,
                                       const Frontier& frontier,
                                       const geometry_msgs::Pose& robot_pose,
                                       const double path_length_gain,
                                       const double orientation_change_gain,
                                       const double frontier_size_gain,
                                       double& cost ) {
    double path_length, ori_change;
    if ( planner_ != NULL ) {
        waypoint_t start, end;
        unsigned int cell = 0;
        map.worldToCell( robot_pose.position.x, robot_pose.position.y, cell );
        map.cellToXY( cell, start.x, start.y );
        map.worldToCell( frontier.pose.position.x, frontier.pose.position.y, cell );
        map.cellToXY( cell, end.x, end.y );
        path_t* path = planner_->planPath( start, end );

        if ( path->size() <= 1 )
            return false; // Frontier unrechable

        path_length = path->size() * map.resolution;

        //ori_change = std::acos( start.x*(*path)[1].x + start.y*(*path)[1].y);
        //ori_change /= std::sqrt( pow( start.x, 2 ) + pow( start.y, 2 ))
                    //    * std::sqrt( pow((*path)[1].x, 2 ) + pow((*path)[1].y, 2 ));
        //ROS_INFO( "Path length: %f Orientation change: %f", path_length, ori_change );
        ori_change = 1.0;
        ROS_INFO( "Start: %d %d First path cell: %d %d", start.x, start.y, (*path)[1].x, (*path)[1].y );
    } else {
        path_length = 1.0;
        ori_change = 0.0; // TODO
    }

    cost = path_length_gain * path_length
            + orientation_change_gain * ori_change
            - frontier_size_gain * frontier.size * map.resolution;
    return true;
}

double ExploreFrontier::getOrientationChange( const Frontier& frontier, const geometry_msgs::Pose& robot_pose ) {
    double robot_yaw = tf::getYaw( robot_pose.orientation );
    double robot_atan2 = atan2(robot_pose.position.y + sin(robot_yaw), robot_pose.position.x + cos(robot_yaw));
    double frontier_atan2 = atan2(frontier.pose.position.x, frontier.pose.position.y);
    return robot_atan2 - frontier_atan2;
}

float ExploreFrontier::getFrontierGain( const Frontier& frontier, double map_resolution ) {
    return frontier.size * map_resolution;
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
        std::vector<geometry_msgs::Pose>& goals,
        double potential_scale,
        double orientation_scale,
        double gain_scale )
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
        if ( getFrontierCost( map, goal.frontier, robot_pose,
                              potential_scale, orientation_scale, gain_scale, goal.cost ))
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

void ExploreFrontier::getVisualizationMarkers(std::vector<Marker>& markers)
{
    Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "frontiers";
    m.type = Marker::ARROW;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 1.0; //0.8;
    m.scale.y = 1.0; //0.6;
    m.scale.z = 1.0;
    m.color.r = 255;
    m.color.g = 0;
    m.color.b = 0;
    m.color.a = 255;
    m.lifetime = ros::Duration(0);

    m.action = Marker::ADD;
    uint id = 0;
    for (uint i=0; i<frontiers_.size(); i++) {
        Frontier frontier = frontiers_[i];
        m.id = id;
        m.pose = frontier.pose;
        markers.push_back(Marker(m));
        id++;
    }

    m.action = Marker::DELETE;
    for (; id < lastMarkerCount_; id++) {
        m.id = id;
        markers.push_back(Marker(m));
    }

    lastMarkerCount_ = markers.size();
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

} // Namespace frontier_explore
