#ifndef PATH_BUILDER_H
#define PATH_BUILDER_H

#include <cslibs_path_planning/geometry/shape.h>
#include "node.h"

class PathBuilder
{
public:
    PathBuilder();

    operator std::vector<path_geom::PathPose>() {
        return build();
    }

    std::vector<path_geom::PathPose> build() const;
    void addPath(const std::vector<path_geom::PathPose>& path);

    void insertTangentPoint(const Segment* start_segment, Eigen::Vector2d start_pt);

    void insertCurveSegment(const Node* node);
    void extendAlongSourceSegment(const Node *node, double length);
    void extendAlongTargetSegment(const Node *node, double length);
    void extendWithStraightTurningSegment(const Vector2d &pt, double length);

private:
    std::vector<path_geom::PathPose> res;
};

#endif // PATH_BUILDER_H
