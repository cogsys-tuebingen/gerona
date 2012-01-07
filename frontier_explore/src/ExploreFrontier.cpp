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
using namespace costmap_2d;

#define MAP_LETHAL_OBSTACLE 50
#define MAP_NO_INFORMATION -1

namespace frontier_explore {

ExploreFrontier::ExploreFrontier() :
    min_frontier_length_( 0.5 ),
        lastMarkerCount_(0),
        planner_(NULL),
        frontiers_()
{
}

ExploreFrontier::~ExploreFrontier()
{ /* Nothing to do */ }

bool ExploreFrontier::getFrontiers( const nav_msgs::OccupancyGrid& map, std::vector<geometry_msgs::Pose>& frontiers ) {
    findFrontiers( map );
    if ( frontiers_.size() == 0 )
        return false;

    frontiers.clear();
    for ( std::size_t i = 0; i < frontiers_.size(); ++i )
        frontiers.push_back( frontiers_[i].pose );

    return (frontiers.size() > 0);
}

float ExploreFrontier::getFrontierCost( const Frontier& frontier ) {
//    ROS_DEBUG("cost of frontier: %f, at position: (%.2f, %.2f, %.2f)", planner_->getPointPotential(frontier.pose.position),
//              frontier.pose.position.x, frontier.pose.position.y, tf::getYaw(frontier.pose.orientation));
    if ( planner_ != NULL )
        return planner_->getPointPotential(frontier.pose.position);
    else
        return 1.0;
}

double ExploreFrontier::getOrientationChange( const Frontier& frontier, const tf::Stamped<tf::Pose>& robot_pose ) {
    double robot_yaw = tf::getYaw(robot_pose.getRotation());
    double robot_atan2 = atan2(robot_pose.getOrigin().y() + sin(robot_yaw), robot_pose.getOrigin().x() + cos(robot_yaw));
    double frontier_atan2 = atan2(frontier.pose.position.x, frontier.pose.position.y);
    double orientation_change = robot_atan2 - frontier_atan2;

//    ROS_DEBUG("Orientation change: %.3f degrees, (%.3f radians)", orientation_change * (180.0 / M_PI), orientation_change);
    return orientation_change;
}

float ExploreFrontier::getFrontierGain( const Frontier& frontier, double map_resolution ) {
    return frontier.size * map_resolution;
}

bool ExploreFrontier::isOpenCell( const nav_msgs::OccupancyGrid& map, unsigned int idx ) const {
    return map.data[idx] != MAP_NO_INFORMATION && map.data[idx] < MAP_LETHAL_OBSTACLE;
}

void ExploreFrontier::cellIdxToPosition( const nav_msgs::OccupancyGrid& map, unsigned int idx, double& x, double &y ) const {
    x = map.info.origin.position.x + map.info.resolution * (double)(idx % map.info.width);
    y = map.info.origin.position.y + map.info.resolution * (double)(idx / map.info.height);
    /*if ( x < 0 ) x += 0.5*map.info.resolution;
    else if ( x > 0 ) x -= 0.5*map.info.resolution;
    if ( y < 0 ) y += 0.5*map.info.resolution;
    else if ( y < 0 ) y -= 0.5*map.info.resolution;*/
}

void ExploreFrontier::findFrontiers( const nav_msgs::OccupancyGrid& map ) {
    frontiers_.clear();

    int w = map.info.width;
    int h = map.info.height;
    unsigned int size = ( w * h );

    bool visited[size];
    memset( visited, false, size*sizeof(bool));
    std::list<unsigned int> toVisit;
    bool left, right, top, bottom;
    std::vector<FrontierPoint> frontier;
    int minIdx, maxIdx;

    // For all cells
    for ( unsigned int i = 0; i < size; ++i ) {
        if ( !isOpenCell( map, i ) || visited[i] )
            continue;

        frontier.clear();
        toVisit.push_back( i );
        minIdx = size;
        maxIdx = -1;
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
            if ( left && map.data[cell-1] == MAP_NO_INFORMATION ) {
                count++;
                dir += btVector3(-1, 0, 0);
            }
            if ( right && map.data[cell+1] == MAP_NO_INFORMATION ) {
                ++count;
                dir += btVector3(1, 0, 0);
            }
            if ( top && map.data[cell-w] == MAP_NO_INFORMATION ) {
                ++count;
                dir += btVector3(0, -1, 0);
            }
            if ( bottom && map.data[cell+w] == MAP_NO_INFORMATION ) {
                ++count;
                dir += btVector3(0, 1, 0);
            }

            visited[cell] = true;
            if ( count == 0 )
                continue;
            frontier.push_back( FrontierPoint( cell, dir / (double)count ));
            if ((int)cell < minIdx )
                minIdx = cell;
            if ((int)cell > maxIdx )
                maxIdx = cell;

            // Current cell is a frontier cell. Visited all open and unvisited neighbor cells
            if ( top && left && !visited[cell-w-1] && isOpenCell( map, cell-w-1 ))
                toVisit.push_back( cell-w-1 );
            if ( top && !visited[cell-w] && isOpenCell( map, cell-w ))
                toVisit.push_back( cell-w );
            if ( top && right && !visited[cell-w+1] && isOpenCell( map, cell-w+1 ))
                toVisit.push_back( cell-w+1 );
            if ( left && !visited[cell-1] && isOpenCell( map, cell-1 ))
                toVisit.push_back( cell-1 );
            if ( right && !visited[cell+1] && isOpenCell( map, cell+1 ))
                toVisit.push_back( cell+1 );
            if ( bottom && left && !visited[cell+w-1] && isOpenCell( map, cell+w-1 ))
                toVisit.push_back( cell+w-1 );
            if ( bottom && !visited[cell+w] && isOpenCell( map, cell+w ))
                toVisit.push_back( cell+w );
            if ( bottom && right && !visited[cell+w+1] && isOpenCell( map, cell+w+1 ))
                toVisit.push_back( cell+w+1 );
        }

        // Frontier found?
        if ( frontier.size() == 0 )
            continue;

        // Check frontier length
        double xMin, yMin, xMax, yMax;
        cellIdxToPosition( map, minIdx, xMin, yMin );
        cellIdxToPosition( map, maxIdx, xMax, yMax);
        if ( sqrt( pow( xMin - xMax, 2 ) + pow( yMin - yMax, 2 )) < min_frontier_length_ )
            continue; // Too short

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
        cellIdxToPosition( map, frontier[middle].idx, x, y );

        // Create and store detected frontier
        Frontier newFrontier;
        newFrontier.pose.position.x = x;
        newFrontier.pose.position.y = y;
        newFrontier.pose.position.z = 0;
        newFrontier.pose.orientation = tf::createQuaternionMsgFromYaw( frontierYaw );
        newFrontier.size = frontier.size();
        frontiers_.push_back( newFrontier );
    }

    /*int idx;
    int w = map.info.width;
    int h = map.info.height;
    int size = ( w * h );
    char mapBuffer[size];

    // Find all frontiers (open cells next to unknown cells)
    for ( idx = 0; idx < size; ++idx ) {
        if ((map.data[idx] < MAP_LETHAL_OBSTACLE && map.data[idx] != MAP_NO_INFORMATION ) &&
                (((idx+1 < size) && (map.data[idx+1] == MAP_NO_INFORMATION)) ||
                    ((idx-1 >= 0) && (map.data[idx-1] == MAP_NO_INFORMATION)) ||
                    ((idx+w < size) && (map.data[idx+w] == MAP_NO_INFORMATION)) ||
                    ((idx-w >= 0) && (map.data[idx-w] == MAP_NO_INFORMATION)))) {
            mapBuffer[idx] = -128;
        } else {
            mapBuffer[idx] = -127;
        }
    }

    // Clean up frontiers detected on separate rows of the map
    // TODO I don't understand this. Wtf?
    idx = h - 1;
    for ( int y = 0; y < w; ++y ) {
        mapBuffer[idx] = -127;
        idx += h;
    }

    // Group adjoining map_ pixels
    int segment_id = 127;
    std::vector< std::vector<FrontierPoint> > segments;
    for ( int i = 0; i < size; ++i ) {
        if ( mapBuffer[i] == -128 ) {
            std::vector<int> neighbors;
            std::vector<FrontierPoint> segment;
            neighbors.push_back(i);

            // Claim all neighbors
            while ( neighbors.size() > 0 ) {
                int idx = neighbors.back();
                neighbors.pop_back();
                mapBuffer[idx] = segment_id;

                btVector3 tot(0,0,0);
                int c = 0;
                if ((idx+1 < size) && (map.data[idx+1] == MAP_NO_INFORMATION)) {
                    tot += btVector3(1,0,0);
                    c++;
                }
                if ((idx-1 >= 0) && (map.data[idx-1] == MAP_NO_INFORMATION)) {
                    tot += btVector3(-1,0,0);
                    c++;
                }
                if ((idx+w < size) && (map.data[idx+w] == MAP_NO_INFORMATION)) {
                    tot += btVector3(0,1,0);
                    c++;
                }
                if ((idx-w >= 0) && (map.data[idx-w] == MAP_NO_INFORMATION)) {
                    tot += btVector3(0,-1,0);
                    c++;
                }
                assert(c > 0);
                segment.push_back(FrontierPoint(idx, tot / c));

                // Consider 8 neighborhood cells
                if (((idx-1) > 0) && (mapBuffer[idx-1] == -128))
                    neighbors.push_back(idx-1);
                if (((idx + 1) < size) && (mapBuffer[idx+1] == -128))
                    neighbors.push_back(idx+1);
                if (((idx - w) > 0) && (mapBuffer[idx-w] == -128))
                    neighbors.push_back(idx - w);
                if (((idx - w + 1) > 0) && (mapBuffer[idx-w+1] == -128))
                    neighbors.push_back(idx - w + 1);
                if (((idx - w - 1) > 0) && (mapBuffer[idx-w-1] == -128))
                    neighbors.push_back(idx - w - 1);
                if (((idx + w) < size) && (mapBuffer[idx+w] == -128))
                    neighbors.push_back(idx + w);
                if (((idx + w + 1) < size) && (mapBuffer[idx+w+1] == -128))
                    neighbors.push_back(idx + w + 1);
                if (((idx + w - 1) < size) && (mapBuffer[idx+w-1] == -128))
                    neighbors.push_back(idx + w - 1);
            }

            segments.push_back(segment);
            segment_id--;
            if ( segment_id < -127 )
                break;
        }
    }

    int num_segments = 127 - segment_id;
    if ( num_segments <= 0 )
        return;

    for ( unsigned int i = 0; i < segments.size(); ++i ) {
        Frontier frontier;
        std::vector<FrontierPoint>& segment = segments[i];
        std::size_t size = segment.size();

        // Check min frontier length
        if ( size * map.info.resolution < min_frontier_length_ )
            continue;

        float x = 0, y = 0;
        btVector3 d(0,0,0);

        for ( std::size_t j = 0; j < size; ++j ) {
            d += segment[j].d;
            int idx = segment[j].idx;
            x += (idx % w);
            y += (idx / w);
        }
        d = d / size;
        frontier.pose.position.x = map.info.origin.position.x + map.info.resolution * (x / size);
        frontier.pose.position.y = map.info.origin.position.y + map.info.resolution * (y / size);
        frontier.pose.position.z = 0.0;

        frontier.pose.orientation = tf::createQuaternionMsgFromYaw(btAtan2(d.y(), d.x()));
        frontier.size = size;

        frontiers_.push_back(frontier);
    }*/

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
