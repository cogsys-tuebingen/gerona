#ifndef PATH_BUILDER_H
#define PATH_BUILDER_H

#include <cslibs_path_planning/geometry/shape.h>
#include <path_msgs/PathSequence.h>
#include "analyzer.h"
#include "node.h"

class PathBuilder : public Analyzer
{
public:
    PathBuilder(Search &search);

    operator path_msgs::PathSequence() {
        return build();
    }

    path_msgs::PathSequence build() const;
    void addPath(const path_msgs::PathSequence& path);

    void insertTangentPoint(const Segment* start_segment, Eigen::Vector2d start_pt);

    void insertCurveSegment(const Node* node);
    void extendAlongSourceSegment(const Node *node, double length);
    void extendAlongTargetSegment(const Node *node, double length);
    void extendWithStraightTurningSegment(const Eigen::Vector2d &pt, double length);

private:
    void insertPose(const Eigen::Vector2d& pt, double c_yaw);

private:
    path_msgs::PathSequence res;
};

#endif // PATH_BUILDER_H
