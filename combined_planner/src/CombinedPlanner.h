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
#include "Path2d.h"

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

    enum State { WAITING_FOR_GMAP, VALID_PATH, GOAL_REACHED };

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
     * @exception PlannerException If the goal or the position of the robot lies
     *      outside of the global map or if there is no global/local map.
     * @exception NoPathException If there is no global path.
     */
    void setGoal( const lib_path::Pose2d& robot_pose, const lib_path::Pose2d& goal );

    /**
     * @brief Select next waypoint if necessary and try to find a local path
     *      to this waypoint. Try to find a new global path if neccesary.
     * @param robot_pose Current position of the robot in map coordinates.
     *      Used to check if we have to select a new waypoint or if we reached the goal.
     * @param force_replan Set this to true if we should caluclate a new local path
     *      even though we didn't select a new waypoint.
     *
     * @throw PlannerException If an error occurred during local path planning.
     * @throw NoPathException If there is no path on the global map.
     */
    void update( const lib_path::Pose2d& robot_pose, bool force_replan = false );

    /**
     * @brief Check if we have reached the goal.
     * @return True if the robot reached the goal. False otherwise.
     */
    bool isGoalReached( const lib_path::Pose2d& robot_pose ) const;

    /**
     * @brief Set the planner to an inactive state.
     */
    void reset();

    /**
     * @brief Check if we've computed a new local path during last update.
     * @return True if there is a new local path.
     */
    bool hasNewLocalPath() const
        { return new_local_path_; }

    /**
     * @brief Set a new map used for global path planning.
     * @attention There is no internal copy of the map! The given pointer should be valid
     *      as long as this object is in use.
     * @attention Call this function only if the map contains new data. We will always allow
     *      global replanning on a new map.
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
    const WaypointList& getLocalPath() const
        { return lplanner_->getPath().getWaypoints(); }

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
     *
     * @exception PlannerException If one of start or goal lies outsied of
     *      the map or if there is no map to plan on.
     * @exception NoPathException If there is no global path.
     */
    void findGlobalPath( const lib_path::Pose2d& start, const lib_path::Pose2d& goal );

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

    /// Flag if we got a new global map since last global replan
    bool new_gmap_;

    /// Current local path
    Path2d lpath_;

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

    /// Flag if we computed a new local path during the last update
    bool new_local_path_;

    /// Current planner state
    State state_;
};

} // namespace combined_planner

#endif // COMBINEDPLANNER_H
