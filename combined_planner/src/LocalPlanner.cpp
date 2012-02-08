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
    : map_( NULL )
{
    // Read/set configuration
    configure();

    // Init reed Shepp planner
    generatePatterns();
}

LocalPlanner::~LocalPlanner()
{ /* Nothing to do */ }

bool LocalPlanner::planPath( const lib_path::Pose2d &start,
                             list<Pose2d>& global_waypoints,
                             const Pose2d& global_goal )
{
    path_.reset();

    // Check input and status
    if ( map_ == NULL ) {
        throw CombinedPlannerException( "No local map!" );
    }

    // Check start pose
    if ( !map_->isInMap( Point2d( start.x, start.y ))) {
        throw CombinedPlannerException( "Start pose outside of the local map." );
    }

    // Clear start pose
    CircleArea clear_area( Point2d( start.x, start.y ), 0.3, map_ );
    map_->setAreaValue( clear_area, (uint8_t)0 );

    // Create a list of reachable global waypoints
    list<list<Pose2d>::iterator> possible_goals;
    list<Pose2d>::iterator global_wp_it = global_waypoints.begin();
    double max_wp_dist = 0.45*map_->getResolution()*(double)( min( map_->getHeight(), map_->getWidth()));
    while ( global_wp_it != global_waypoints.end()) {
        if ( global_wp_it->distance_to( start ) <= 0.8*max_wp_dist ) {
            possible_goals.push_front( global_wp_it );
            global_wp_it++;
        } else
            break;
    }

    // If we didn't find a possible global waypoint but the goal is out of reach
    bool goal_in_area = global_goal.distance_to( start ) <= max_wp_dist;
    if ( possible_goals.empty() && !goal_in_area )
        throw CombinedPlannerException( "Found no possible local goal and the global goal is too far away." );

    // If the goal is reachable try to find a path to it (no sampling!)
    Curve* curve = NULL;
    if ( goal_in_area ) {
        // The Reed Shepp planner needs cell coordinates
        Pose2d start_cell = pos2map( start, *map_ );
        Pose2d end_cell = pos2map( global_goal, *map_ );
        curve = rs_.find_path( start_cell, end_cell, map_ );

        // Remove all waypoints on success
        if ( curve && curve->is_valid()) {
            global_waypoints.clear();
            generatePath( curve );
            delete curve;
            return path_.getWaypointCount() > 0;
        }
    }

    // Search a path to the selected waypoints. Latest selected one first
    SamplingPlanner s_planner( &rs_, map_ );
    while ( !possible_goals.empty()) {
        LocalWaypointRegion wp_region( *possible_goals.front(), 0.25, 10.0 );
        curve = s_planner.createPath( start, &wp_region, 0 );

        // Remove waypoints on success
        if ( curve && curve->is_valid()) {
            global_waypoints.erase( global_waypoints.begin(), ++possible_goals.front());
            generatePath( curve );
            return path_.getWaypointCount() > 0;
            break;
        }

        possible_goals.pop_front();
    }

    // Fail. No local path found
    if ( curve != NULL )
        delete curve;
    return false;
}

void LocalPlanner::setMap( lib_path::GridMap2d *map )
{   
    map_ = map;
}

void LocalPlanner::generatePath( lib_path::Curve *curve )
{
    path_.reset();
    curve->reset_iteration();
    while( curve->has_next()) {
        path_.addWaypoint( map2pos( curve->next(), *map_ ));
    }
}

void LocalPlanner::configure()
{
    /// @todo Remove ROS dependency from this class
    ros::NodeHandle n( "~/reed_shepp/" );

    // Read config
    double circle_radius, wp_distance, cost_backw, cost_forw, cost_curve, cost_straight;
    n.param<double> ("circle_radius", circle_radius, 1.0f);
    n.param<double> ("max_waypoint_distance", wp_distance, 0.25f);
    n.param<double> ("cost_backwards", cost_backw, 5.0f);
    n.param<double> ("cost_forwards", cost_forw, 1.0f);
    n.param<double> ("cost_curve", cost_curve, 1.5f);
    n.param<double> ("cost_straight", cost_straight, 1.0f);

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
