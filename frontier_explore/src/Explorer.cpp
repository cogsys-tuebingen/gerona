
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include "Explorer.h"
#include "FrontierFinder.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

Explorer::Explorer()
    : goal_selector_( NULL ), min_frontier_length_( 1.0 )
{
}

Explorer::~Explorer()
{
    if ( goal_selector_ != NULL )
        delete goal_selector_;
}

bool Explorer::calculateGoals(
        lib_path::GridMap2d *explore_map,
        lib_path::GridMap2d *ground_map,
        Eigen::Vector3d &robot_pose,
        std::vector<Goal> &poi )
{
    // Find frontiers
    std::vector<Frontier> frontiers;
    FrontierFinder::getFrontiers( explore_map, min_frontier_length_, frontiers );

    // Initialize goal selector
    if ( goal_selector_ == NULL )
        goal_selector_ = new GoalSelector( ground_map );
    else {
        goal_selector_->reset();
        goal_selector_->setMap( ground_map );
    }
    goal_selector_->setPathLengthGain( path_length_gain_ );
    goal_selector_->setOrientationChangeGain( orientation_change_gain_ );
    goal_selector_->setImportanceGain( importance_gain_ );

    // Convert frontiers to goals and add them
    // We are using the size of a frontier as importance factor
    size_t size = frontiers.size();
    for ( size_t i = 0; i < size; ++i )
        goal_selector_->addGoal( Goal( frontiers[i].pose, frontiers[i].size * explore_map->getResolution()), robot_pose );


    // Add the given POIs
    goal_selector_->addGoals( poi, robot_pose );

    // Calculate sorted list of goals
    goals_.clear();
    goal_selector_->getSortedGoals( goals_ );

    return goals_.size() > 0;
}

} // namespace
