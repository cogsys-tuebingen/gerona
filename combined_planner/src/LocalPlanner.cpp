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
#include "LocalWaypointRegion.h"

using namespace std;
using namespace lib_path;
using namespace combined_planner;

LocalPlanner::LocalPlanner()
    : map_( NULL ),
      robot_bubble_( 0.35 )
{
    // Read/set configuration
    configure();

    // Init reed Shepp planner
    generatePatterns();
}

LocalPlanner::~LocalPlanner()
{ /* Nothing to do */ }

void LocalPlanner::planPath( const lib_path::Pose2d &start,
                             list<Pose2d>& global_waypoints,
                             const Pose2d& global_goal )
{
    path_.reset();

    // Got a map?
    if ( map_ == NULL ) {
        throw PlannerException( "No local map!" );
    }

    // Start pose should be inside of the map
    if ( !map_->isInMap( Point2d( start.x, start.y ))) {
        throw PlannerException( "Start pose outside of the local map." );
    }

    // Clear start pose to avoid planner errors if we are very close to an obstacle
    CircleArea clear_area( Point2d( start.x, start.y ), robot_bubble_, map_ );
    map_->setAreaValue( clear_area, (uint8_t)0 );

    // Create a list of reachable global waypoints.
    // A waypoint is reachable if it lies WELL inside of the map bounds
    /// @todo Don't assume a quadratic map here and don't assume, that the start is equal to the map origin
    list<list<Pose2d>::iterator> possible_goals;
    list<Pose2d>::iterator global_wp_it = global_waypoints.begin();
    double max_wp_dist = 0.3*map_->getResolution()*(double)( min( map_->getHeight(), map_->getWidth()));
    while ( global_wp_it != global_waypoints.end()) {
        if ( global_wp_it->distance_to( start ) <= max_wp_dist ) {
            possible_goals.push_front( global_wp_it );
            global_wp_it++;
        } else
            break;
    }

    // Fallback: If there are waypoints left but no one was selected, take any waypoint inside of the map
    /// @todo Do we need this?
    if ( possible_goals.empty() && !global_waypoints.empty()) {
        global_wp_it = global_waypoints.begin();
        while ( global_wp_it != global_waypoints.end()) {
            if ( map_->isInMap( *global_wp_it )) {
                possible_goals.push_front( global_wp_it );
            }
            global_wp_it++;
        }
    }

    // If we didn't find a possible global waypoint but the goal is out of reach
    bool goal_in_area = start.distance_to( global_goal ) <= max_wp_dist;
    if ( possible_goals.empty() && !goal_in_area )
        throw NoPathException( "Found no local waypoint and the global goal lies outside of the local map." );

    // If the goal is reachable try to find a path to it (no sampling!)
    Curve* curve = NULL;
    if ( goal_in_area ) {
        // The Reed Shepp planner needs cell coordinates
        Pose2d start_cell = pos2map( start, *map_ );
        Pose2d end_cell = pos2map( global_goal, *map_ );
        curve = rs_.find_path( start_cell, end_cell, map_ );

        // Check resulting path
        if ( curve && curve->is_valid()) {
            //if ( !path_.isFree( map_, start, robot_bubble_ ))
                //throw PlannerException( "New local path is not free. This should never happen!" );

            // Remove all global waypoints on success
            global_waypoints.clear();
            generatePath( curve );
            delete curve;
            if ( path_.getWaypointCount() > 0 )
                return;
        }
    }

    // Search a path to an area around the selected waypoints. Latest selected one first
    SamplingPlanner s_planner( &rs_, map_ );
    while ( !possible_goals.empty()) {
        // Allow to reach waypoints backwards only if they are close enough!
        // Otherwise we may drive large distances backwards since we wont't
        // replan before we are close to the end of the path.
        LocalWaypointRegion wp_region( *possible_goals.front(), 0.25, 10.0, possible_goals.size() < 2 );
        curve = s_planner.createPath( start, &wp_region, 0 );

        // Check resulting path
        if ( curve && curve->is_valid()) {
            //if ( !path_.isFree( map_, start, 0.3 ))
               // throw PlannerException( "New local path is not free. This should never happen!" );

            // Remove preceding waypoints on success
            global_waypoints.erase( global_waypoints.begin(), ++possible_goals.front());
            generatePath( curve );
            if ( path_.getWaypointCount() > 0 )
                return;
        }

        // Try next waypoint
        possible_goals.pop_front();
    }

    // Fail. No local path found
    if ( curve != NULL )
        delete curve;
    throw NoPathException( "No local path found." );
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
    n.param<double> ("cost_backwards", cost_backw, 3.0f);
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
