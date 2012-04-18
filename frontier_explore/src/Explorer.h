#ifndef EXPLORER_H
#define EXPLORER_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Workspace

// Project
#include "GoalSelector.h"

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

/**
 * @brief Calculates exploration goals.
 */
class Explorer
{
public:
    /**
     * @brief Initializes the object
     */
    Explorer();

    virtual ~Explorer();

    /**
     * @brief Calculate exploration goals.
     * @param explore_map Map used to find forntiers.
     * @param ground_map Map used for goal cost calculation (e.g. path planning)
     * @param robot_pose Position of the robot in (ground) map coordinates (x, y, theta).
     * @param poi Externally calculated points of interest in (ground) map coordinates.
     * These goals will be includes into the goal selection.
     * @return True if at least on goal was found.
     */
    bool calculateGoals(
            lib_path::GridMap2d *explore_map,
            lib_path::GridMap2d *ground_map,
            Eigen::Vector3d &robot_pose,
            std::vector<Goal> &poi );

    /**
     * @brief Return the calculated goals
     * @param goals Will be filled with all goals
     * @param return True if at least one goal exists
     */
    bool getGoals( std::vector<WeightedGoal> &goals ) {
        goals.assign( goals_.begin(), goals_.end());
        return goals_.size() > 0;
    }

    /// Set goal distance gain in cost calculation
    void setPathLengthGain( const double gain ) { path_length_gain_ = gain; }

    /// Set orientation change gain in cost calculation
    void setOrientationChangeGain( const double gain ) { orientation_change_gain_ = gain; }

    /// Set importance gain in cost calculation
    void setImportanceGain( const double gain ) { importance_gain_ = gain; }

    /// Set minimum side length od frontier bounding box
    void setMinFrontierLength( const double min_length ) { min_frontier_length_ = min_length; }

private:

    /// Used to select the goals
    GoalSelector *goal_selector_;

    /// Minimum side length of frontier bounding box
    double min_frontier_length_;

    /// @todo remove these members

    /// Gain of distance to the goal in cost calculation
    double path_length_gain_;

    /// Gain of orientation change in cost calculation
    double orientation_change_gain_;

    /// Gain of importance in cost calculation
    double importance_gain_;

    /// All goals. Best goal first
    std::vector<WeightedGoal> goals_;
};

} // namespace

#endif // EXPLORER_H
