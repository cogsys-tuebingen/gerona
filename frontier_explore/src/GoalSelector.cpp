
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>

// Workspace
#include <utils/LibUtil/MathHelper.h>

// Project
#include "GoalSelector.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

GoalSelector::GoalSelector( lib_path::GridMap2d *map )
    : planner_( NULL )
{
    setMap( map );
}

GoalSelector::~GoalSelector()
{
    if ( planner_ != NULL )
        delete planner_;
}

void GoalSelector::addGoal( const Goal &goal, const Eigen::Vector3d &robot_pose )
{
    // Robot pose on map?
    if ( !map_->isInMap( robot_pose.x(), robot_pose.y()))
        return;

    // Clear robot position on the map
    //lib_path::CircleArea robot_bubble( lib_path::Point2d( robot_pose.x(), robot_pose.y()), 0.35, map_ );
    lib_path::CircleBuffer robot_bubble( lib_path::Point2d( robot_pose.x(), robot_pose.y()), 0.35, map_ );
    map_->getAreaValues( robot_bubble );
    map_->setAreaValue( robot_bubble, 0 );

    // Plan path etc
    double path_length, ori_change;
    if ( planner_ != NULL ) {
        // Plan a path from the current position to the goal
        lib_path::waypoint_t start, end;
        unsigned int x, y;
        if ( !map_->point2cell( robot_pose.x(), robot_pose.y(), x, y ))
            return;
        start.x = x; start.y = y;
        if ( !map_->point2cell( goal.pose.x(), goal.pose.y(), x, y ))
            return;
        end.x = x; end.y = y;
        bool path_exists = planner_->planPath( start, end );

        // Goal is reachable?
        if ( !path_exists )
            return;

        // Calculate frontier cost parameters
        lib_path::path_t *path = planner_->getLastPath();
        path_length = path->size()*map_->getResolution(); // Well, this is an approximation...

        // The difference between the robot orientation and the goal orientation is
        // important if the robot is close to the frontier. It is not so important
        // if the frontier is far away. In this case we check the difference between the
        // robot orientation and the start of the planned path
        double robot_yaw = robot_pose(2);
        if ( path_length < 2.0 || path->size() < 2 ) {
            ori_change = fabs( MathHelper::AngleDelta( robot_yaw, goal.pose(2)));
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
    WeightedGoal weighted_goal;
    weighted_goal.cost = path_length_gain_ * path_length
            + orientation_change_gain_ * ori_change
            - importance_gain_ * goal.importance;
    weighted_goal.dist = path_length;
    weighted_goal.goal = goal;

    // Add goal
    goals_.push_back( weighted_goal );

    // Reset map
    /// @todo always reset the map
    map_->setAreaValue( robot_bubble );
}

void GoalSelector::addGoals( const std::vector<Goal> &goals, const Eigen::Vector3d &robot_pose )
{
    size_t size = goals.size();
    for ( size_t i = 0; i < size; ++i )
        addGoal( goals[i], robot_pose );
}

bool GoalSelector::getSortedGoals( std::vector<WeightedGoal> &goals )
{
    goals.assign( goals_.begin(), goals_.end());
    std::sort( goals.begin(), goals.end());
    return goals.size() > 0;
}

void GoalSelector::setMap( lib_path::GridMap2d *map )
{
    map_ = map;
    if ( planner_ == NULL )
        planner_ = new lib_path::AStar( map );
    else
        planner_->setNewMap( map );
}

} // namespace
