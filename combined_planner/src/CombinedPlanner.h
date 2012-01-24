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

    virtual ~CombinedPlanner();

    /**
     * @brief Set a new map used for global path planning.
     * @attention There is no internal copy of the map! The given pointer should be valid
     *      as long as this object is in use.
     * @param gmap The new global map.
     */
    virtual void setGlobalMap( lib_path::GridMap2d* gmap );

    virtual bool setGoal( const lib_path::Pose2d& robot_pose, const lib_path::Pose2d& goal );

    /**
     * @brief Get the latest raw global path.
     * The raw path was not minimized or flattened.
     * @param path_raw The raw path will be written to this list.
     */
    virtual void getGlobalPathRaw( std::list<lib_path::Point2d>& path_raw ) const
        { gplanner_->getLatestPathRaw( path_raw ); }

    /**
     * @brief Get the latest flattened global path.
     * @param path The path will be written to this list.
     */
    virtual void getGlobalPath( std::list<lib_path::Point2d>& path ) const
        { gplanner_->getLatestPath( path ); }

    /**
     * @brief Get the waypoints from the latest global path.
     * The first entry of that list is always the current waypoint we
     * are trying to reach. The last entry is the goal pose. If we
     * reached the goal pose, the result will be empty.
     * @param The latest global waypoints.
     */
    virtual const std::list<lib_path::Pose2d>& getGlobalWaypoints() const
        { return gwaypoints_; }

    /**
     * @brief Set a new map used for local path planning.
     * @attention There is no internal copy of the map! The given pointer should be valid
     *      as long as this object is in use.
     * @param gmap The new local map.
     */
    virtual void setLocalMap( lib_path::GridMap2d* lmap );

    /**
     * @brief Select next waypoint if necessary and try to find a local path
     *      to this waypoint.
     * @param robot_pose Current position of the robot in map coordinates.
     *      Used to check if we have to select a new waypoint.
     * @param force_replan Set this to true if we should caluclate a new local path
     *      even though we didn't select a new waypoint.
     * @return True if there is a new local path. False otherwise.
     * @throw CombinedPlannerException If an error occurred during local path planning.
     */
    virtual bool updateLocalPath( const lib_path::Pose2d& robot_pose, bool force_replan = false );

    /**
     * @brief Get the latest local path.
     * @return the current local waypoints.
     */
    virtual const std::list<lib_path::Pose2d>& getLocalWaypoints() const
        { return lplanner_->getPath(); }

    /**
     * @brief Check if we have reached the goal.
     * @param robot_pose The current robot position in map coordinates.
     * @return True if reached the goal. False otherwise.
     */
    virtual bool isGoalReached( const lib_path::Pose2d &robot_pose ) const;

    /**
     * @brief Remove the current goal from the waypoint list which
     *      indicates that the goal is reached.
     */
    virtual void setGoalReached();

protected:

    /**
     * @brief Try to find a global path and calculate waypoints.
     * @param start Start of the path.
     * @param goal End of the path.
     * @return True if there is a path. False otherwise.
     * @exception CombinedPlannerException If one of start or goal lies outsied of
     *      the map or if there is no map to plan on.
     */
    virtual bool findGlobalPath( const lib_path::Pose2d& start, const lib_path::Pose2d& goal );

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
     * @return True id the waypoint is reached.
     */
    virtual bool isWaypointReached( const lib_path::Pose2d &robot_pose, const lib_path::Pose2d& wp ) const;

    virtual void getPoseDelta( const lib_path::Pose2d& p, const lib_path::Pose2d& q,
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

    /// Maximum goal distance error. Used to determine if we have reached the goal
    double goal_dist_eps_;

    /// Maximum goal angle error. Used to determine if we have reached the goal
    double goal_angle_eps_;

    /// Maximum distance to the next waypoint. Used to determine if we have to select the next waypoint
    double wp_dist_eps_;

    /// Maximum angle delta to the next waypoint. Used to determine if we have to select the next waypoint
    double wp_angle_eps_;

    /// End of the latest local path
    lib_path::Pose2d latest_lgoal_;

    /// Global goal
    lib_path::Pose2d ggoal_;

    /// Flag if we reached the latest global goal
    bool ggoal_reached_;
};

} // namespace combined_planner

#endif // COMBINEDPLANNER_H
