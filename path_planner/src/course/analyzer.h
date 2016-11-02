#ifndef ANALYZER_H
#define ANALYZER_H


#include "node.h"
#include <string>
#include <ros/node_handle.h>
#include <cslibs_path_planning/geometry/shape.h>

class Search;

class Analyzer
{
public:
    friend class Search;

    Analyzer(Search& search);

    Eigen::Vector2d findStartPointOnSegment(const Node* node) const;
    Eigen::Vector2d findStartPointOnSegment(const Node* node, const Transition* transition) const;
    Eigen::Vector2d findEndPointOnSegment(const Node* node) const;
    Eigen::Vector2d findEndPointOnSegment(const Node* node, const Transition* transition) const;

    double calculateStraightCost(Node* current_node, const Eigen::Vector2d &start_point_on_segment, const Eigen::Vector2d &end_point_on_segment) const;
    double calculateCurveCost(Node* current_node) const;
    double calculateEffectiveLengthOfNextSegment(const Node* node) const;

    bool isSegmentForward(const Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target) const;
    bool isNextSegmentForward(const Node* node) const;
    bool isPreviousSegmentForward(const Node* current_node) const;
    bool isStartSegmentForward(const Node *node) const;

    std::string signature(const Node* head) const;

private:
    ros::NodeHandle pnh_;

    Search& search;

    double backward_penalty_factor;
    double turning_straight_segment;
    double turning_penalty;

};

#endif // ANALYZER_H
