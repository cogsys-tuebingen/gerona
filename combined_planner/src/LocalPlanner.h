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
#include "PlannerExceptions.h"
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
     *
     * @param start The start pose of the path (usually the position of the robot) in map/world coordinates.
     * @param global_waypoints A list containing global waypoints in map/world coordinates.
     *      Try to find a path to one of this waypoints and remove all
     *      preceding waypoints on success.
     * @param global_goal The current global goal in map/world coordinates. Try to
     *      reach the global goal if there are no waypoints left or if the goal is reachable.
     *
     * @throws PlannerException If there is no map or if the start/end pose lies
     *      outside of the map.
     * @throw NoPathException If we didn't find a valid path. A valid path contains at least one waypoint.
     */
    void planPath( const lib_path::Pose2d& start,
                   std::list<lib_path::Pose2d> &global_waypoints,
                   const lib_path::Pose2d &global_goal );

    /**
     * @brief Get the latest path.
     * @return The latest path in map/world coordinates.
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

    /// Radius of the free circle around the robot. We are ignoring obstacles
    /// inside of this circle to avoid planner errors if we are very close to an obstacle.
    double robot_bubble_;
};

} // namespace

#endif // LOCALPLANNER_H
