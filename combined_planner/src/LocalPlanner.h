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
     * @brief Create and configurae object.
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
     * @brief Try to find a local car-like feasible path.
     * @param start The start pose of the path.
     * @param end The (optimal) end pose of the path.
     * @return True if there is a valid path.
     * @throws CombinedPlannerException If there is no map or if the start/end pose lies
     *      outside of the map.
     */
    bool planPath( const lib_path::Pose2d& start, const lib_path::Pose2d& end );

    /**
     * @brief Get the latest path.
     * @return The latest path.
     */
    const std::list<lib_path::Pose2d>& getPath() const
        { return path_; }

protected:
    /**
     * @brief Initialized the Reed Shepp planner.
     */
    void generatePatterns();

    /**
     * @brief Generate a path with poses in map coordinates.
     * @param curve The result of the Reed Shepp planner (cell coordinates...).
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
    std::list<lib_path::Pose2d> path_;

    /// Weight of the last Reed Shepp curve
    double last_weight_;

    /// Used to clear the cells near the robots position
    lib_path::CircleArea* clear_area_;

};

} // namespace

#endif // LOCALPLANNER_H
