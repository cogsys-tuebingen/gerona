#include "analyzer.h"
#include "transition.h"
#include "node.h"
#include "segment.h"
#include "search.h"

Analyzer::Analyzer(Search &search)
    : search(search)
{
}

Analyzer::~Analyzer()
{
}

Eigen::Vector2d Analyzer::findStartPointOnSegment(const Node* node) const
{
    if(node->next_segment == search.start_segment) {
        return search.start_pt;

    } else {
        return findStartPointOnSegment(node, node->transition);
    }
}

Eigen::Vector2d Analyzer::findStartPointOnSegment(const Node* node, const Transition* transition) const
{
    if(node->curve_forward) {
        return transition->path.back();
    } else {
        return transition->path.front();
    }
}


Eigen::Vector2d Analyzer::findEndPointOnSegment(const Node* node) const
{
    if(node->next_segment == search.end_segment) {
        return search.end_pt;

    } else if(!node->next) {
        return node->curve_forward ? node->next_segment->line.endPoint() : node->next_segment->line.startPoint();

    } else  {
        const Node* next_node = node->next;
        return findEndPointOnSegment(next_node, next_node->transition);
    }
}


Eigen::Vector2d Analyzer::findEndPointOnSegment(const Node* node, const Transition* transition) const
{
    if(node->curve_forward) {
        return transition->path.front();
    } else {
        return transition->path.back();
    }
}

bool Analyzer::isPreviousSegmentForward(const Node* node) const
{
    if(node->prev) {
        return isNextSegmentForward(node->prev);
    } else {
        return isStartSegmentForward(node);
    }
}

bool Analyzer::isStartSegmentForward(const Node* node) const
{
    return isSegmentForward(search.start_segment, search.start_pt, findEndPointOnSegment(node, node->transition));
}

bool Analyzer::isNextSegmentForward(const Node* node) const
{
    return isSegmentForward(node->next_segment, findStartPointOnSegment(node), findEndPointOnSegment(node));
}
double Analyzer::calculateEffectiveLengthOfNextSegment(const Node* node) const
{
    return (findStartPointOnSegment(node) - findEndPointOnSegment(node)).norm();
}



bool Analyzer::isSegmentForward(const Segment* segment,
                                const Eigen::Vector2d& pos,
                                const Eigen::Vector2d& target) const
{
    Eigen::Vector2d segment_dir = segment->line.endPoint() - segment->line.startPoint();
    Eigen::Vector2d move_dir = target - pos;
    ROS_DEBUG_STREAM("check segment " <<
                    segment->line.startPoint()(0) << ", " << segment->line.startPoint()(1) <<
                    " -> " <<
                    segment->line.endPoint()(0) << ", " << segment->line.endPoint()(1) <<
                    " with " <<
                    pos(0) << ", " << pos(1) <<
                    " -> " <<
                    target(0) << ", " << target(1));
    if(move_dir.norm() < 0.1) {
        ROS_WARN_STREAM_THROTTLE(1, "effective segment size is small: " << move_dir.norm() <<
                                 ". This can result in wrong paths!");
    }
    return segment_dir.dot(move_dir) >= 0.0;
}

std::string Analyzer::signature(const Node *head) const
{
    std::string res = "";

    const Node* first = head;
    const Node* tmp = head;
    while(tmp) {
        if(tmp->next_segment) {
            std::string dir = isNextSegmentForward(tmp) ? ">" : "<";
            res = dir + res;
        } else {
            res = std::string("?") + res;
        }

        tmp = tmp->prev;
        if(tmp) {
            first = tmp;
        }
    }

    Eigen::Vector2d  end_point_on_start_segment = first->curve_forward ? first->transition->path.front() : first->transition->path.back();
    std::string start_sym = isSegmentForward(search.start_segment, search.start_pt, end_point_on_start_segment) ? ">" : "<";

    return start_sym + res;
}
