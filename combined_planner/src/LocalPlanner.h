/**
 * @file LocalPlanner.h
 * @date Jan 2012
 * @author marks
 */

#ifndef LOCALPLANNER_H
#define LOCALPLANNER_H

// C/C++
#include <vector>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>
#include <utils/LibPath/ReedsShepp/CurveGenerator.h>

class LocalPlanner
{
public:
    LocalPlanner();

    void setMap( lib_path::GridMap2d* map );
    bool planPath( const lib_path::Pose2d& start, const lib_path::Pose2d& end );

protected:
    void generatePatterns();
    void generatePath( lib_path::Curve* curve );

    lib_path::CurveGenerator rs_;
    bool got_map_;
    lib_path::GridMap2d* map_;
    lib_path::MapInfo map_info_;
    std::vector<lib_path::Pose2d> path_;
    double last_weight_;

};

#endif // LOCALPLANNER_H
