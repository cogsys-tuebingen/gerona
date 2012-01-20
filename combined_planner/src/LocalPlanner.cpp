/**
 * @file LocalPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <ros/ros.h>

// Workspace
#include <utils/LibPath/common/MapMath.h>

// Project
#include "LocalPlanner.h"

using namespace std;
using namespace lib_path;

LocalPlanner::LocalPlanner( ros::NodeHandle& n )
    : got_map_( false ),
      map_( NULL )
{
    // Read config
    double circle_radius, wp_distance, cost_backw, cost_forw, cost_curve, cost_straight;
    n.param<double> ("circle_radius", circle_radius, 1.0f);
    n.param<double> ("max_waypoint_distance", wp_distance, 0.25f );
    n.param<double> ("cost_backwards", cost_backw, 3.0f);
    n.param<double> ("cost_forwards", cost_forw, 1.0f);
    n.param<double> ("cost_curve", cost_curve, 1.2f);
    n.param<double> ("cost_straight", cost_straight, 1.0f);

    // Set config
    rs_.set_circle_radius( circle_radius );
    rs_.set_max_waypoint_distance( wp_distance );
    rs_.set_cost_backwards( cost_backw );
    rs_.set_cost_forwards( cost_forw );
    rs_.set_cost_curve( cost_curve );
    rs_.set_cost_straight( cost_straight );

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
    Pose2d start_map = pos2map( start, *map_ );
    Pose2d end_map = pos2map( end, *map_ );
    Curve* curve = rs_.find_path( start_map, end_map, map_ );

    // Check result
    if ( curve && curve->is_valid()) {
        generatePath( curve );
        ROS_INFO( "Found local path! Weight: %f", curve->weight());
        last_weight_ = curve->weight();
        delete curve;
        return true;
    } else {
        ROS_WARN( "No local path found." );
        if ( curve )
            delete curve;
        return false;
    }
}

void LocalPlanner::setMap( lib_path::GridMap2d *map )
{
    map_ = map;
    got_map_ = true;
}

void LocalPlanner::generatePath( lib_path::Curve *curve )
{
    path_.clear();
    curve->reset_iteration();
    while( curve->has_next()) {
        path_.push_back( map2pos( curve->next(), *map_ ));
    }
}

void LocalPlanner::getPath( list<lib_path::Pose2d> &path ) const
{
    path.clear();
    path.assign( path_.begin(), path_.end());
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
