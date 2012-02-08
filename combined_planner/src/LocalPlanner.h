/**
 * @file LocalPlanner.h
 * @date Jan 2012
 * @author marks
 */

#ifndef LOCALPLANNER_H
#define LOCALPLANNER_H

// C/C++
#include <vector>
#include <list>

// ROS
#include <ros/ros.h>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>
#include <utils/LibPath/ReedsShepp/CurveGenerator.h>

// Project
#include "Path2d.h"

namespace combined_planner {

/**
 * @class LocalPlanner
 * @brief Used to calculate local car-like feasible paths.
 * The local planning is based on Reed Shepp curves.
 */
class LocalPlanner
{
public:
    /**
     * @brief Create and configure object.
     */
    LocalPlanner();

    virtual ~LocalPlanner();

    /**
     * @brief Set the current (local) map.
     * @attention There is no internal copy of the given map. The pointer should be valid
     *      as long as this object is in use.
     * @param map The new map.
     */
    void setMap( lib_path::GridMap2d* map );

    /**
     * @brief Try to find a car-like feasible path.
     * @param start The start pose of the path.
     * @param end The (optimal) end pose of the path.
     * @return True if there is a valid path and if this path contains at least one waypoint. False otherwise.
     * @throws CombinedPlannerException If there is no map or if the start/end pose lies
     *      outside of the map.
     */
    bool planPath( const lib_path::Pose2d& start,
                   std::list<lib_path::Pose2d> &global_waypoints,
                   const lib_path::Pose2d &global_goal );

    /**
     * @brief Get the latest path.
     * @return The latest path.
     */
    const Path2d& getPath() const
        { return path_; }

protected:
    /**
     * @brief Initialized the Reed Shepp planner.
     */
    void generatePatterns();

    /**
     * @brief Generate a path with poses in map coordinates.
     * @param curve The result of the Reed Shepp planner (cell coordinates).
     */
    void generatePath( lib_path::Curve* curve );

    /**
     * @brief Read and set the configuration.
     */
    void configure();

    /// Local Reed Shepp planner
    lib_path::CurveGenerator rs_;

    /// The current map. NULL if there is none
    lib_path::GridMap2d* map_;

    /// The latest path
    Path2d path_;
};

} // namespace

#endif // LOCALPLANNER_H