/**
 * @file LocalWaypointRegion.h
 * @date Jan 2012
 * @author marks
 */

#ifndef LOCALWAYPOINTREGION_H
#define LOCALWAYPOINTREGION_H

// C/C++
#include <list>

// Workspace
#include <utils/LibPath/sampling/GoalRegion.h>

/**
 * @class LocalWaypointRegion
 * @brief A simple goal region that creates goals that are shifted
 *      to the right and the left relative to a center pose.
 */
class LocalWaypointRegion : public lib_path::GoalRegion
{
public:
    /**
     * @brief Create object and generate goals.
     * The list of all possible goals will include the given center pose.
     * the orientation of the shifted poses will be equal to the orientation of the
     * center pose.
     * @param center Center pose.
     * @param dist_step Distance the goals will be moved to the left and right
     *      relative to the center pose.
     */
    LocalWaypointRegion( const lib_path::Pose2d &center,
                         const double dist_step,
                         const double angle_steg_deg,
                         const bool inverse = false );

    /* Inherited from GoalRegion */
    virtual void init( unsigned samples_num = 0 ) { /* We have always three samples */ }
    virtual bool getNextGoal( lib_path::Pose2d &goal, double& gain );

protected:

    /**
     * @brief Generate all goals.
     * @param center Center pose.
     * @param dist_step Distance the goals will be moved to the left and right
     *      relative to the center pose.
     */
    virtual void generateGoals( const lib_path::Pose2d &center,
                                const double dist_step,
                                const double angle_steg_deg,
                                const bool inverse );

    /// List of possible goals
    std::list<std::pair<lib_path::Pose2d, double> > goal_list_;
};

#endif // LOCALWAYPOINTREGION_H
