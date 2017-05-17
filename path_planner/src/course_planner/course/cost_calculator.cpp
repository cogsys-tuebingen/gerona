#include "cost_calculator.h"
#include "transition.h"
#include "node.h"
#include "segment.h"
#include "search.h"

CostCalculator::CostCalculator(Search &search)
    : Analyzer(search),

      pnh_("~")
{
    pnh_.param("course/penalty/backwards", backward_penalty_factor, 2.5);
    pnh_.param("course/penalty/turn", turning_penalty, 5.0);

    pnh_.param("course/turning/straight", turning_straight_segment, 0.7);
}

double CostCalculator::calculateStraightCost(Node* node, const Eigen::Vector2d& start_point_on_segment, const Eigen::Vector2d& end_point_on_segment) const
{
    double cost = 0.0;
    bool segment_forward = isSegmentForward(node->next_segment, start_point_on_segment, end_point_on_segment);
    double distance_to_end = (end_point_on_segment - start_point_on_segment).norm();
    if(segment_forward) {
        cost += distance_to_end;
    } else {
        cost += backward_penalty_factor * distance_to_end;
    }

    bool prev_segment_forward = isPreviousSegmentForward(node);
    if(prev_segment_forward != segment_forward) {
        // single turn
        cost += turning_straight_segment;
        cost += turning_penalty;

    } else if(segment_forward != node->curve_forward) {
        // double turn
        cost += 2 * turning_straight_segment;
        cost += 2 * turning_penalty;
    }

    return cost;
}

double CostCalculator::calculateCurveCost(Node *node) const
{
    if(node->curve_forward) {
        return node->transition->arc_length();
    } else {
        return backward_penalty_factor * node->transition->arc_length();
    }
}



