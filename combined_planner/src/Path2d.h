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

class Path2d
{
public:
    Path2d();

    Path2d( const WaypointList &wp );

    void setWaypoints( const WaypointList& wp );

    void updateWaypoints( const double radius, const lib_path::Pose2d &robot_pose );

    bool isEndReached( const lib_path::Pose2d& robot_pose, double max_d_dist = 0.1, double max_d_theta = 0.17 ) const;

    bool isFree( double radius, const lib_path::GridMap2d* map ) const;

    void reset();

    void addWaypoint( const lib_path::Pose2d& wp );

    const WaypointList& getWaypoints() const
        { return waypoints_; }

    void getWaypoints( WaypointList& dest ) const
        { dest.assign( waypoints_.begin(), waypoints_.end()); }

    lib_path::Pose2d getStart() const
        { return start_; }

    void getStart( lib_path::Pose2d& dest ) const
        { dest = start_; }

    lib_path::Pose2d getEnd() const
        { return end_; }

    void getEnd( lib_path::Pose2d& dest ) const
        { dest = end_; }

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
