#ifndef COST_CALCULATOR_H
#define COST_CALCULATOR_H

#include "node.h"
#include "analyzer.h"
#include <string>
#include <ros/node_handle.h>
#include <cslibs_path_planning/geometry/shape.h>

class Search;

class CostCalculator : public Analyzer
{
public:
    friend class Search;

    CostCalculator(Search& search);

    double calculateStraightCost(Node* current_node, const Eigen::Vector2d &start_point_on_segment, const Eigen::Vector2d &end_point_on_segment) const;
    double calculateCurveCost(Node* current_node) const;

private:
    ros::NodeHandle pnh_;

    double backward_penalty_factor;
    double turning_straight_segment;
    double turning_penalty;

};

#endif // COST_CALCULATOR_H
