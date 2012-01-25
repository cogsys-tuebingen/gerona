/**
 * @file LocalPlanner.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <ros/ros.h>

// Workspace
#include <utils/LibPath/common/MapMath.h>
#include <utils/LibPath/sampling/SamplingPlanner.h>

// Project
#include "LocalPlanner.h"
#include "CombinedPlannerException.h"
#include "LocalWaypointRegion.h"

using namespace std;
using namespace lib_path;
using namespace combined_planner;

LocalPlanner::LocalPlanner()
    : map_( NULL ),
      clear_area_( NULL )
{
    // Read/set configuration
    configure();

    // Init reed Shepp planner
    generatePatterns();
}

LocalPlanner::~LocalPlanner()
{
    if ( clear_area_ != NULL )
        delete clear_area_;
}

bool LocalPlanner::planPath( const lib_path::Pose2d &start, const lib_path::Pose2d &end )
{
    last_weight_ = -1.0;
    path_.clear();

    // Check input and status
    if ( map_ == NULL ) {
        throw CombinedPlannerException( "No map!" );
    }

    if ( !map_->isInMap( Point2d( start.x, start.y ))
         || !map_->isInMap( Point2d( end.x, end.y ))) {
        throw CombinedPlannerException( "Start or goal pose outside of the map." );
    }

    // Clear robot pose
    map_->setAreaValue( *clear_area_, (uint8_t)0 );

    // Search a path
    Curve* curve = NULL;
    LocalWaypointRegion goalRegion( end, 0.25, 20.0 );
    SamplingPlanner s_planner( &rs_, map_ );
    curve = s_planner.createPath( start, &goalRegion, 0 );

    // Check result & generate path if there is one
    if ( curve  && curve->is_valid()) {
        generatePath( curve );
        last_weight_ = curve->weight();
        delete curve;
        return true;
    }

    if ( curve != NULL ) /// @todo Is this neccessary?
        delete curve;
    return false;
}

void LocalPlanner::setMap( lib_path::GridMap2d *map )
{   
    map_ = map;
    if ( clear_area_ == NULL )
        clear_area_ = new CircleArea( Point2d( 0, 0 ), 0.3, map_ );
}

void LocalPlanner::generatePath( lib_path::Curve *curve )
{
    path_.clear();
    curve->reset_iteration();
    while( curve->has_next()) {
        path_.push_back( map2pos( curve->next(), *map_ ));
    }
}

void LocalPlanner::configure()
{
    /// @todo Remove ROS dependency from this class
    ros::NodeHandle n( "~/reed_shepp/" );

    // Read config
    double circle_radius, wp_distance, cost_backw, cost_forw, cost_curve, cost_straight;
    n.param<double> ("circle_radius", circle_radius, 1.0f);
    n.param<double> ("max_waypoint_distance", wp_distance, 0.25f );
    n.param<double> ("cost_backwards", cost_backw, 1.5f);
    n.param<double> ("cost_forwards", cost_forw, 1.0f);
    n.param<double> ("cost_curve", cost_curve, 120.0f);
    n.param<double> ("cost_straight", cost_straight, 100.0f);

    // Set config
    rs_.set_circle_radius( circle_radius );
    rs_.set_max_waypoint_distance( wp_distance );
    rs_.set_cost_backwards( cost_backw );
    rs_.set_cost_forwards( cost_forw );
    rs_.set_cost_curve( cost_curve );
    rs_.set_cost_straight( cost_straight );
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
