#ifndef GOALSELECTOR_H
#define GOALSELECTOR_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <utils/LibPath/a_star/AStar.h>

// Project
#include "FrontierFinder.h"

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

/// Represents a goal
struct Goal
{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Position and orientation (x, y, theta) in map coordinates
    Eigen::Vector3d pose;

    /// Importance of the goal. Should be > 0
    double importance;

    /**
     * @brief Initializes all fields
     * @param p Goal pose (x, y, theta) in map coordinates
     * @param i importance value. Should be > 0.
     */
    Goal( const Eigen::Vector3d &p, const double i ) : pose( p ), importance( i ) {}

    Goal() : pose( 0, 0, 0 ), importance( 0 ) {}

};

/// Represents a weighted goal
struct WeightedGoal
{
    /// The goal
    Goal goal;

    /// Cost of the goal (might be < 0)
    double cost;

    /// Estimated distance to the goal (map units, usually meter)
    double dist;

    /**
     * @brief Initializes all fields
     * @param g The goal
     * @param cost Cost of the goal > 0
     * @param d Distance to the goal (in map units, usually meters)
     */
    WeightedGoal( const Goal &g, const double c, const double d ) : goal( g ), cost( c ), dist( d ) {}

    WeightedGoal() : cost( 1.0 ), dist( 1.0 ) {}

    bool operator<( const WeightedGoal& o ) const { return cost < o.cost; }
};

/**
 * @brief Calculates the cost of given goals and sorts them.
 */
class GoalSelector
{
public:

    /**
     * @param map The map we plan paths on.
     * @attention There is no internal copy of the map!
     */
    GoalSelector( lib_path::GridMap2d *map );

    virtual ~GoalSelector();

    /**
     * @brief Set the map used for path planning.
     * @param map The map
     * @attention There is no internal copy of the map!
     */
    void setMap( lib_path::GridMap2d *map );

    /**
     * @brief Delete all goals.
     */
    void reset() {
        goals_.clear();
    }

    /**
     * @brief Add a new goal.
     * @param goal The goal.
     * @param robot_pose Position of the robot in map coordinates (x, y, theta).
     */
    void addGoal( const Goal &goal, const Eigen::Vector3d &robot_pose );

    /**
     * @param Add an array of goals.
     * @param goals The goals to add.
     */
    void addGoals( const std::vector<Goal> &goals, const Eigen::Vector3d &robot_pose );

    /**
     * @brief Get the weighted goals. All returned goals are reachable.
     * @param goals Will be filled with the weighted goals. Best goal first.
     * @return True if at least one goal is reachable.
     */
    bool getSortedGoals( std::vector<WeightedGoal> &goals );

    /// Set goal distance gain in cost calculation
    virtual void setPathLengthGain( const double gain ) { path_length_gain_ = gain; }

    /// Set orientation change gain in cost calculation
    virtual void setOrientationChangeGain( const double gain ) { orientation_change_gain_ = gain; }

    /// Set importance gain in cost calculation
    virtual void setImportanceGain( const double gain ) { importance_gain_ = gain; }

private:

    /// Current map used for path calculation
    lib_path::GridMap2d *map_;

    /// List of all reachable goals. Not always sorted!
    std::vector<WeightedGoal> goals_;

    /// Gain of distance to the goal in cost calculation
    double path_length_gain_;

    /// Gain of orientation change in cost calculation
    double orientation_change_gain_;

    /// Gain of importance in cost calculation
    double importance_gain_;

    /// Planner used to determine if a goal is reachable and to calculate the distance to it
    lib_path::AStar* planner_;
};

} // namespace

#endif // GOALSELECTOR_H
