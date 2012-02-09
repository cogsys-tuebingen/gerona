/**
 * @file Path2d.h
 * @date Feb 2012
 * @author marks
 */

#ifndef PATH2D_H
#define PATH2D_H

// C/C++
#include <list>

// Workspace
#include <utils/LibPath/common/Pose2d.h>
#include <utils/LibPath/common/GridMap2d.h>

namespace combined_planner {

typedef std::list<lib_path::Pose2d> WaypointList;

/**
 * @class Path2d
 * @brief Represents a 2d path and offeres several methods to check
 * if the end is reached or if there are obstacles in our way.
 */
class Path2d
{
public:

    /**
     * @brief Create an empty path.
     * Start and end pose will be zero.
     */
    Path2d();

    /**
     * @brief Create a path from the given waypoint list.
     * @param wp List of waypoints. The first entry is the start pose, the last one
     *      the end pose. Size of the list should be >= 2.
     */
    Path2d( const WaypointList &wp );

    /**
     * @brief Set the waypoints of this path.
     * @param wp List of waypoints. The first entry is the start pose, the last one
     *      the end pose. Size of the list should be >= 2.
     */
    void setWaypoints( const WaypointList& wp );

    /**
     * @brief Remove all reached waypoints.
     * @param radius Remove all waypoints with a distance to the robot less than
     *      this value. The iteration starts with the first entry of the waypoint list
     *      and ends on the first waypoint, that is not inside of the given radius.
     * @param robot_pose Current pose of the robot.
     */
    void updateWaypoints( const double radius, const lib_path::Pose2d &robot_pose );

    /**
     * @brief Check if the end of the path is reached.
     * @param robot_pose Current pose of the robot.
     * @param max_d_dist Maximum distance deviation.
     * @param max_d_theta Maximum orientation deviation.
     * @return True if the end of the path is reached.
     */
    bool isEndReached( const lib_path::Pose2d& robot_pose, double max_d_dist = 0.1, double max_d_theta = 0.17 ) const;

    /**
     * @brief
     */
    bool isFree( const lib_path::GridMap2d* map, const lib_path::Pose2d robot_pose, const double skip_radius = 0 ) const;

    /**
     * @brief Check if the waypoints are obstacle free.
     * @param radius Check a circular area with this radius for each waypoints.
     * @param map The map.
     * @return False if there is an obstacle.
     */
    bool areWaypointsFree( const double radius, const lib_path::GridMap2d* map ) const;

    /**
     * @brief Reset the object.
     * Remove all waypoints, set start and end to zero.
     */
    void reset();

    /**
     * @brief Add a new waypoints.
     * If the waypoint list is empty, the given pose will be the start pose this path.
     * If the list contains at least one entry, the given
     * pose will be the new end of the path.
     * @param wp The new waypoint.
     */
    void addWaypoint( const lib_path::Pose2d& wp );

    /**
     * @brief Get the remaining waypoints.
     * The start of the path is the first entry.
     * @return The remaining waypoints.
     */
    const WaypointList& getWaypoints() const
        { return waypoints_; }

    /**
     * @brief Get the waypoints.
     * The start of the path is the first entry.
     * @param dest Write the waypoints to this list.
     */
    void getWaypoints( WaypointList& dest ) const
        { dest.assign( waypoints_.begin(), waypoints_.end()); }

    /**
     * @brief Get the start pose.
     * The start pose of the path won't change if you remove
     * already reached waypoints.
     * @return The beginning of the path.
     */
    lib_path::Pose2d getStart() const
        { return start_; }

    /**
     * @brief Get the start pose.
     * The start pose of the path won't change if you remove
     * already reached waypoints.
     * @param dest Write the start pose to this parameter.
     */
    void getStart( lib_path::Pose2d& dest ) const
        { dest = start_; }

    /**
     * @brief Get the end of the path.
     * @return The end of the path.
     */
    lib_path::Pose2d getEnd() const
        { return end_; }

    /**
     * @brief Get the end of the path.
     * @param dest Write the end pose the this parameter.
     */
    void getEnd( lib_path::Pose2d& dest ) const
        { dest = end_; }

    /**
     * @brief Get the number of remaining waypoints.
     * @return The number of waypoints.
     */
    int getWaypointCount() const
        { return waypoints_.size(); }

private:

    /// Start of the path
    lib_path::Pose2d start_;

    /// End of the path
    lib_path::Pose2d end_;

    /// Waypoints of this path
    WaypointList waypoints_;

};

} // namespace combined_planner

#endif // PATH2D_H
