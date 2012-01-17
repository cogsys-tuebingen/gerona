/**
 * @file GlobalPlanner.h
 * @date Jan 2012
 * @author marks
 */

#ifndef GLOBALPLANNER_H
#define GLOBALPLANNER_H

// C/C++
#include <vector>

// Workspace
#include <utils/LibPath/a_star/AStar.h>

class GlobalPlanner
{
public:
    GlobalPlanner( lib_path::GridMap2d* map );

    virtual void setMap( lib_path::GridMap2d* map );
    virtual bool planPath( const lib_path::Point2d& start, lib_path::Point2d& goal );
    virtual void getLatestPathRaw( std::vector<lib_path::Point2d>& path ) const;
    virtual void getLatestPath( std::vector<lib_path::Point2d>& path ) const;

protected:

    virtual bool isLineFree( const lib_path::waypoint_t& p1,
                             const lib_path::waypoint_t& p2 ) const;

private:
    /// The map we are working on
    lib_path::GridMap2d* map_;

    /// A* search object
    lib_path::AStar a_star_;

    /// Start of path in map coordinates
    lib_path::Point2d start_;

    /// Goal in map coordinates
    lib_path::Point2d goal_;

    /// Latest planned path
    std::vector<lib_path::Point2d> path_;

    /// Latest planned path (not flattened etc)
    std::vector<lib_path::Point2d> path_raw_;
};

#endif // GLOBALPLANNER_H
