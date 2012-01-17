/**
 * @file LocalPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <ros/ros.h>

// Project
#include "LocalPlanner.h"

using namespace std;
using namespace lib_path;

LocalPlanner::LocalPlanner()
    : got_map_( false ),
      map_( NULL )
{
    // Init reed Shepp planner
    generatePatterns();
}

bool LocalPlanner::planPath( const lib_path::Pose2d &start, const lib_path::Pose2d &end )
{
    // Delete old path
    path_.clear();
    last_weight_ = -1.0;

    // Check input and status
    if ( !got_map_ ) {
        ROS_ERROR( "Cannot plan a local path. No map!" );
        return false;
    }

    if ( !map_->isInMap( Point2d( start.x, start.y ))
         || !map_->isInMap( Point2d( end.x, end.y ))) {
        ROS_ERROR( "Got invalid start/end pose for local path planning. Pose not in map!" );
        return false;
    }

    // Search a path
    Curve* curve = rs_.find_path( start, end, &map_info_ );

    // Check result
    if ( curve->is_valid()) {
        generatePath( curve );
        last_weight_ = curve->weight();
        return true;
    } else {
        ROS_WARN( "No local path found." );
        return false;
    }
}

void LocalPlanner::setMap( lib_path::GridMap2d *map )
{
    map_info_.width = map->getWidth();
    map_info_.height = map->getHeight();
    map_info_.resolution = map->getResolution();
    map_info_.origin = map->getOrigin();
    //
}

void LocalPlanner::generatePath( lib_path::Curve *curve )
{
    path_.clear();
    curve->reset_iteration();
    while( curve->has_next()) {
        /// @todo We might have to transform the point into the map system?
        path_.push_back( curve->next() );
    }
}

void LocalPlanner::generatePatterns()
{
    rs_.parse("LSL");
    rs_.parse("LSR");
    rs_.parse("RSL");
    rs_.parse("RSR");

    rs_.parse("|LSL");
    rs_.parse("|LSR");
    rs_.parse("|RSL");
    rs_.parse("|RSR");

    rs_.parse("|L|SL");
    rs_.parse("|L|SR");
    rs_.parse("|R|SL");
    rs_.parse("|R|SR");

    rs_.parse("LS|L");
    rs_.parse("LS|R");
    rs_.parse("RS|L");
    rs_.parse("RS|R");

    rs_.parse("L|SL");
    rs_.parse("L|SR");
    rs_.parse("R|SL");
    rs_.parse("R|SR");

    rs_.parse("L|S|L");
    rs_.parse("L|S|R");
    rs_.parse("R|S|L");
    rs_.parse("R|S|R");

    rs_.parse("|LS|L");
    rs_.parse("|LS|R");
    rs_.parse("|RS|L");
    rs_.parse("|RS|R");

    rs_.parse("|L|S|L");
    rs_.parse("|L|S|R");
    rs_.parse("|R|S|L");
    rs_.parse("|R|S|R");

    rs_.parse("LRL");
    rs_.parse("RLR");

    rs_.parse("L|R|L");
    rs_.parse("R|L|R");
    rs_.parse("|L|R|L");
    rs_.parse("|R|L|R");

    rs_.parse("LR|L");
    rs_.parse("RL|R");
    rs_.parse("|LR|L");
    rs_.parse("|RL|R");

    rs_.parse("L|RL");
    rs_.parse("R|LR");
    rs_.parse("|L|RL");
    rs_.parse("|R|LR");

    rs_.parse("LR(b)|L(b)R");
    rs_.parse("|LR(b)|L(b)R");
    rs_.parse("RL(b)|R(b)L");
    rs_.parse("|RL(b)|R(b)L");

    rs_.parse("L|R(b)L(b)|R");
    rs_.parse("R|L(b)R(b)|L");
    rs_.parse("|L|R(b)L(b)|R");
    rs_.parse("|R|L(b)R(b)|L");
}
