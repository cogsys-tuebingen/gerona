/**
 * @file CombinedPlanner.h
 * @date Jan 2012
 * @author marks
 */

#ifndef COMBINEDPLANNER_H
#define COMBINEDPLANNER_H

// ROS
#include <ros/ros.h>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>

// Project
#include "LocalPlanner.h"
#include "GlobalPlanner.h"

namespace combined_planner {

/**
 * @class CombinedPlanner
 * @brief A path planner that uses A* for global planning and Reed Shepp curves for
 *      local, car-like feasible paths.
 * The resulting path won't be optimal and there might be a path even if this
 * planner doesn't find one.
 */
class CombinedPlanner
{
public:
    /**
     * @brief Create the object.
     * Initializes all pointers to NULL.
     */
    CombinedPlanner();

    ~CombinedPlanner();

    /**
     * @brief Set a new goal and plan a path to this goal.
     * This method tries to find a global path and calculates waypoints
     * as well as an initial local path.
     * @exception CombinedPlannerException If the goal or the position of the robot lies
     *      outside of the global map, if there is no global/local map or if there
     *      is no local path to the first waypoint.
     */
    void setGoal( const lib_path::Pose2d& robot_pose, const lib_path::Pose2d& goal );

    /**
     * @brief Select next waypoint if necessary and try to find a local path
     *      to this waypoint. Try to find a new global path if neccesary.
     * @param robot_pose Current position of the robot in map coordinates.
     *      Used to check if we have to select a new waypoint or if we reached the goal.
     * @param force_replan Set this to true if we should caluclate a new local path
     *      even though we didn't select a new waypoint.
     * @return True if there is a new local path. False otherwise.
     * @throw CombinedPlannerException If an error occurred during local path planning.
     */
    void update( const lib_path::Pose2d& robot_pose, bool force_replan = false );

    /**
     * @brief Check if we have reached the goal.
     * @return True if the robot reached the goal. False otherwise.
     */
    bool isGoalReached( const lib_path::Pose2d& robot_pose ) const;

    bool hasValidPath() const
        { return valid_path_; }

    bool hasNewLocalPath() const
        { return new_local_path_; }

    /**
     * @brief Set a new map used for global path planning.
     * @attention There is no internal copy of the map! The given pointer should be valid
     *      as long as this object is in use.
     * @param gmap The new global map.
     */
    void setGlobalMap( lib_path::GridMap2d* gmap );

    /**
     * @brief Set a new map used for local path planning.
     * @attention There is no internal copy of the map! The given pointer should be valid
     *      as long as this object is in use.
     * @param gmap The new local map.
     */
    void setLocalMap( lib_path::GridMap2d* lmap );

    /**
     * @brief Get the latest local path.
     * @return the current local path.
     */
    const std::list<lib_path::Pose2d>& getLocalPath() const
        { return lplanner_->getPath(); }

    /**
     * @brief Get the latest raw global path.
     * The raw path was not minimized or flattened.
     * @param path_raw The raw path will be written to this list.
     */
    void getGlobalPathRaw( std::list<lib_path::Point2d>& path_raw ) const
        { gplanner_->getLatestPathRaw( path_raw ); }

    /**
     * @brief Get the latest flattened global path.
     * @param path The path will be written to this list.
     */
    void getGlobalPath( std::list<lib_path::Point2d>& path ) const
        { gplanner_->getLatestPath( path ); }

    /**
     * @brief Get the waypoints from the latest global path.
     * The first entry of that list is always the current waypoint we
     * are trying to reach. The last entry is the goal pose. If we
     * reached the goal pose, the result will be empty.
     * @param The latest global waypoints.
     */
    const std::list<lib_path::Pose2d>& getGlobalWaypoints() const
        { return gwaypoints_; }

protected:

    /**
     * @brief Try to find a global path and calculate waypoints.
     * @param start Start of the path.
     * @param goal End of the path.
     * @return True if there is a path. False otherwise.
     * @exception CombinedPlannerException If one of start or goal lies outsied of
     *      the map or if there is no map to plan on.
     */
    bool findGlobalPath( const lib_path::Pose2d& start, const lib_path::Pose2d& goal );

    bool findPathToWaypoint( const lib_path::Pose2d& start );

    bool findPathToGoal( const lib_path::Pose2d& start );

    /**
     * @brief Calculate waypoints from a given path.
     * This method is used to convert the result of an A* search into
     * a list of waypoints that are feasible for local path planning.
     * Each waypoint will point to the next one and the result will always
     * contain the given goal pose.
     * @param path The path in map-coordinates containing points not poses.
     * @param goal The current goal pose.
     * @param waypoints The result will be written to this parameter.
     */
    void calculateWaypoints( const std::vector<lib_path::Point2d> &path,
                             const lib_path::Pose2d &goal,
                             list<lib_path::Pose2d> &waypoints ) const;

    /**
     * @brief Check if a waypoint is reached.
     * A waypoint is reached if the distance of the robot and the angle delta is
     * smaller than the maximum errors allowed.
     * @param robot_pose Current position of the robot in map coordinates.
     * @param wp The waypoint in map coordinates.
     * @return True if the waypoint is reached.
     */
    bool isWaypointReached( const lib_path::Pose2d &robot_pose, const lib_path::Pose2d& wp ) const;

    void getPoseDelta( const lib_path::Pose2d& p, const lib_path::Pose2d& q,
                                 double& d_dist, double& d_theta ) const;

    lib_path::Pose2d getNormalizedDelta( const lib_path::Point2d &start, const lib_path::Point2d &end ) const;

    /// Map used for local path planning
    lib_path::GridMap2d* lmap_;

    /// Map used for global path planning
    lib_path::GridMap2d* gmap_;

    /// Planner used for local paths
    LocalPlanner* lplanner_;

    /// Planner used for global paths
    GlobalPlanner* gplanner_;

    /// Global waypoints
    std::list<lib_path::Pose2d> gwaypoints_;

    /// Global goal
    lib_path::Pose2d ggoal_;

    /// Start of the latest global path
    lib_path::Pose2d gstart_;

    /// Flag if we reached the latest global goal
    bool ggoal_reached_;

    /// End of the latest local path
    lib_path::Pose2d lgoal_;

    /// Start of the latest local path
    lib_path::Pose2d lstart_;

    /// Maximum goal distance error. Used to determine if we have reached the goal
    double goal_dist_eps_;

    /// Maximum goal angle error. Used to determine if we have reached the goal
    double goal_angle_eps_;

    /// Maximum distance to the next waypoint. Used to determine if we have to select the next waypoint
    double wp_dist_eps_;

    /// Maximum angle delta to the next waypoint. Used to determine if we have to select the next waypoint
    double wp_angle_eps_;

    /// Plan a new local path at least every N meter
    double local_replan_dist_;

    /// Plan a new local path if the orientation changed N radian
    double local_replan_theta_;

    /// Flag if there is a valid global and a valid local path
    bool valid_path_;

    /// Flag if we computed a new local path during the last update
    bool new_local_path_;
};

} // namespace combined_planner

#endif // COMBINEDPLANNER_H
