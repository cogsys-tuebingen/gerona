#include "search.h"

#include <utils_path/common/CollisionGridMap2d.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <set>
#include <tf/tf.h>

#include "course_generator.h"

#include "near_course_test.hpp"

namespace {
bool comp (const Node* lhs, const Node* rhs)
{
    return lhs->cost<rhs->cost;
}
}

Search::Search(const CourseGenerator& generator)
    : pnh_("~"), generator_(generator)
{    
    pnh_.param("size/forward", size_forward, 0.4);
    pnh_.param("size/backward", size_backward, -0.6);
    pnh_.param("size/width", size_width, 0.5);

    pnh_.param("course/penalty/backwards", backward_penalty_factor, 2.5);
    pnh_.param("course/penalty/turn", turning_penalty, 5.0);

    pnh_.param("course/turning/straight", turning_straight_segment, 0.7);

    std::string map_service = "/static_map";
    pnh_.param("map_service",map_service, map_service);

    map_service_client_ = pnh_.serviceClient<nav_msgs::GetMap> (map_service);
}


std::vector<path_geom::PathPose> Search::findPath(const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose)
{

    nav_msgs::GetMap map_service;
    if(!map_service_client_.exists()) {
        map_service_client_.waitForExistence();
    }
    if(!map_service_client_.call(map_service)) {
        ROS_ERROR("map service lookup failed");
        return {};
    }

    initMaps(map_service.response.map);

    if(!findAppendices(map_service.response.map, start_pose, end_pose)) {
        return {};
    }


    if(start_segment == end_segment) {
        PathBuilder path_builder;
        path_builder.addPath(start_appendix);

        path_builder.insertTangentPoint(start_segment, start_pt);
        path_builder.insertTangentPoint(end_segment, end_pt);

        path_builder.addPath(end_appendix);

        return path_builder;
    }

    return performDijkstraSearch();
}

std::vector<path_geom::PathPose> Search::performDijkstraSearch()
{
    initNodes();

    std::set<Node*, bool(*)(const Node*, const Node*)> priority_queue(&comp);

    enqueueStartingNodes(priority_queue);

    min_cost = std::numeric_limits<double>::infinity();

    while(!priority_queue.empty()) {
        Node* current_node = *priority_queue.begin();
        priority_queue.erase(priority_queue.begin());

        if(current_node->next_segment == end_segment) {
            generatePathCandidate(current_node);
            continue;
        }

        for(int i = 0; i <= 1; ++i) {
            const auto& transitions =  i == 0 ? current_node->next_segment->forward_transitions : current_node->next_segment->backward_transitions;
            for(const Transition& next_transition : transitions) {
                Node* neighbor = &nodes.at(&next_transition);

                double curve_cost = calculateCurveCost(current_node);
                double straight_cost = calculateStraightCost(current_node, findStartPointOnNextSegment(current_node), findEndPointOnSegment(current_node, &next_transition));

                double new_cost = current_node->cost + curve_cost + straight_cost;

                if(new_cost < neighbor->cost) {
                    neighbor->cost = new_cost;

                    neighbor->prev = current_node;
                    current_node->next = neighbor;

                    if(priority_queue.find(neighbor) != priority_queue.end()) {
                        priority_queue.erase(neighbor);
                    }
                    priority_queue.insert(neighbor);
                }
            }
        }
    }

    PathBuilder path_builder;
    path_builder.addPath(start_appendix);
    path_builder.addPath(best_path);
    path_builder.addPath(end_appendix);

    return path_builder;
}

void Search::enqueueStartingNodes(std::set<Node*, bool(*)(const Node*, const Node*)>& queue)
{
    for(int i = 0; i <= 1; ++i) {
        const auto& transitions = i == 0 ? start_segment->forward_transitions : start_segment->backward_transitions;
        for(const Transition& next_transition : transitions) {

            Node* node = &nodes.at(&next_transition);

            // distance from start_pt to transition
            Eigen::Vector2d  end_point_on_segment = node->curve_forward ? next_transition.path.front() : next_transition.path.back();
            node->cost = calculateStraightCost(node, start_pt, end_point_on_segment);
            queue.insert(node);
        }
    }
}

void Search::initMaps(const nav_msgs::OccupancyGrid& map)
{
    unsigned w = map.info.width;
    unsigned h = map.info.height;

    bool replace = !map_info  ||
            map_info->getWidth() != w ||
            map_info->getHeight() != h;

    if(replace){
        map_info.reset(new lib_path::CollisionGridMap2d(map.info.width, map.info.height, tf::getYaw(map.info.origin.orientation), map.info.resolution, size_forward, size_backward, size_width));
    }

    std::vector<uint8_t> data(w*h);
    int i = 0;

    /// Map data
    /// -1: unknown -> 0
    /// 0:100 probabilities -> 1 - 100
    for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
        data[i++] = std::min(100, *it + 1);
    }

    map_info->setLowerThreshold(50);
    map_info->setUpperThreshold(70);
    map_info->setNoInformationValue(-1);

    map_info->set(data, w, h);
    map_info->setOrigin(lib_path::Point2d(map.info.origin.position.x, map.info.origin.position.y));
}

void Search::initNodes()
{
    nodes.clear();
    for(const Segment& s : generator_.getSegments()) {
        for(const Transition& t : s.forward_transitions) {
            nodes[&t].transition = &t;
            nodes[&t].curve_forward = true;
            nodes[&t].next_segment = t.target;
        }
        for(const Transition& t : s.backward_transitions) {
            nodes[&t].transition = &t;
            nodes[&t].curve_forward = false;
            nodes[&t].next_segment = t.source;
        }
    }
}

bool Search::findAppendices(const nav_msgs::OccupancyGrid& map, const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose)
{
    ROS_WARN_STREAM("searching appendices");

    start_appendix = findAppendix<AStarPatsyForward, AStarPatsyForwardTurning>(map, start_pose, "start");
    if(start_appendix.empty()) {
        return false;
    }

    path_geom::PathPose start = start_appendix.back();
    start_segment = generator_.findClosestSegment(start, M_PI / 8, 0.5);
    start_pt = start_segment->line.nearestPointTo(start.pos_);
    if(!start_segment) {
        ROS_ERROR_STREAM("cannot find a path for start pose " << start.pos_);
        return false;
    }


    end_appendix = findAppendix<AStarPatsyReversed, AStarPatsyReversedTurning>(map, end_pose, "end");
    if(end_appendix.empty()) {
        return false;
    }
    std::reverse(end_appendix.begin(), end_appendix.end());

    path_geom::PathPose end = end_appendix.front();
    end_segment = generator_.findClosestSegment(end, M_PI / 8, 0.5);
    end_pt = end_segment->line.nearestPointTo(end.pos_);
    if(!end_segment) {
        ROS_ERROR_STREAM("cannot find a path for  end pose " << end.pos_);
        return false;
    }

    return true;
}

template <typename AlgorithmForward, typename AlgorithmFull>
std::vector<path_geom::PathPose> Search::findAppendix(const nav_msgs::OccupancyGrid &map, const path_geom::PathPose& pose, const std::string& type)
{
    lib_path::Pose2d pose_map = convertToMap(pose);

    AlgorithmForward algo_forward;
    algo_forward.setMap(map_info.get());

    NearCourseTest<AlgorithmForward> goal_test_forward(generator_, algo_forward, map, map_info.get());
    auto path_start = algo_forward.findPath(pose_map, goal_test_forward, 0);
    if(path_start.empty()) {
        ROS_WARN_STREAM("cannot connect to " << type << " without turning");
        AlgorithmFull algo_forward_turn;
        NearCourseTest<AlgorithmFull> goal_test_forward_turn(generator_, algo_forward_turn, map, map_info.get());
        algo_forward_turn.setMap(map_info.get());
        path_start = algo_forward_turn.findPath(pose_map, goal_test_forward_turn, 0);
        if(path_start.empty()) {
            ROS_ERROR_STREAM("cannot connect to " << type);
            return {};
        }
    }

    std::vector<path_geom::PathPose> appendix;
    for(const auto& node : path_start) {
        appendix.push_back(convertToWorld(node));
    }

    return appendix;
}

Eigen::Vector2d Search::findStartPointOnNextSegment(const Node* node) const
{
    if(node->next_segment == start_segment) {
        return start_pt;

    } else {
        return findStartPointOnNextSegment(node, node->transition);
    }
}

Eigen::Vector2d Search::findStartPointOnNextSegment(const Node* node, const Transition* transition) const
{
    if(node->curve_forward) {
        return transition->path.back();
    } else {
        return transition->path.front();
    }
}


Eigen::Vector2d Search::findEndPointOnNextSegment(const Node* node) const
{
    if(node->next_segment == end_segment) {
        return end_pt;

    } else if(!node->next) {
        return node->curve_forward ? node->next_segment->line.endPoint() : node->next_segment->line.startPoint();

    } else  {
        const Node* next_node = node->next;
        return findEndPointOnSegment(next_node, next_node->transition);
    }
}


Eigen::Vector2d Search::findEndPointOnSegment(const Node* node, const Transition* transition) const
{
    if(node->curve_forward) {
        return transition->path.front();
    } else {
        return transition->path.back();
    }
}




bool Search::isPreviousSegmentForward(const Node* node) const
{
    if(node->prev) {
        return isNextSegmentForward(node->prev);
    } else {
        return isStartSegmentForward(node);
    }
}

bool Search::isStartSegmentForward(const Node* node) const
{
    return isSegmentForward(start_segment, start_pt, findEndPointOnSegment(node, node->transition));
}

bool Search::isNextSegmentForward(const Node* node) const
{
    return isSegmentForward(node->next_segment, findStartPointOnNextSegment(node), findEndPointOnNextSegment(node));
}
double Search::effectiveLengthOfNextSegment(const Node* node) const
{
    return (findStartPointOnNextSegment(node) - findEndPointOnNextSegment(node)).norm();
}



bool Search::isSegmentForward(const Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target) const
{
    Eigen::Vector2d segment_dir = segment->line.endPoint() - segment->line.startPoint();
    Eigen::Vector2d move_dir = target - pos;
    if(move_dir.norm() < 0.1) {
        ROS_WARN_STREAM("effective segment size is small: " << move_dir.norm());
    }
    return segment_dir.dot(move_dir) >= 0.0;
}


path_geom::PathPose Search::convertToWorld(const NodeT& node)
{
    double tmpx, tmpy;

    map_info->cell2pointSubPixel(node.x, node.y, tmpx, tmpy);

    return path_geom::PathPose(tmpx, tmpy, node.theta/* - tf::getYaw(map.info.origin.orientation)*/);
}

lib_path::Pose2d  Search::convertToMap(const path_geom::PathPose& pt)
{
    lib_path::Pose2d res;
    unsigned tmpx, tmpy;
    map_info->point2cell(pt.pos_(0), pt.pos_(1), tmpx, tmpy);
    res.x = tmpx;
    res.y = tmpy;
    res.theta = pt.theta_;// - map_info->getRotation();
    return res;
}


double  Search::calculateStraightCost(Node* node, const Eigen::Vector2d& start_point_on_segment, const Eigen::Vector2d& end_point_on_segment) const
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

double Search::calculateCurveCost(Node *node) const
{
    if(node->curve_forward) {
        return node->transition->arc_length();
    } else {
        return backward_penalty_factor * node->transition->arc_length();
    }
}

void Search::generatePathCandidate(Node* node)
{
    // finish the path -> connect to end point
    double additional_cost = calculateStraightCost(node, findStartPointOnNextSegment(node), end_pt);
    node->cost += additional_cost;

    ROS_WARN_STREAM("found candidate with signature " << signature(node) << " with cost " << node->cost);
    if(node->cost < min_cost) {
        min_cost = node->cost;
        best_path.clear();

        std::deque<const Node*> transitions;
        Node* tmp = node;
        while(tmp) {
            transitions.push_front(tmp);
            if(tmp->prev) {
                tmp->prev->next = tmp;
            }
            tmp = tmp->prev;
        }

        PathBuilder path_builder;
        generatePath(transitions, path_builder);
        best_path = path_builder;
    }
}

void Search::generatePath(const std::deque<const Node*>& path_transitions, PathBuilder& path_builder) const
{
    path_builder.insertTangentPoint(start_segment, start_pt);

    bool segment_forward = isStartSegmentForward(path_transitions.front());

    ROS_INFO_STREAM("generating path from " << path_transitions.size() << " transitions");
    for(std::size_t i = 0, n = path_transitions.size(); i < n; ++i) {
        const Node* current_node = path_transitions[i];

        double eff_len = effectiveLengthOfNextSegment(current_node);
        if(eff_len < std::numeric_limits<double>::epsilon()) {
            // the segment has no length -> only insert the transition curve
            path_builder.insertCurveSegment(current_node);
            continue;
        }

        bool next_segment_forward = isNextSegmentForward(current_node);

        // insert turning point?
        if(next_segment_forward == segment_forward) {
            // same direction

            if(current_node->curve_forward == next_segment_forward) {
                // no straight segment necessary
                path_builder.insertCurveSegment(current_node);

            } else {
                // double turn
                /*
                 * The direction effectively stays the same, only the transition is in the opposite direction
                 * -> two turns are performed -> two straight segments are needed
                 */

                if(current_node->curve_forward) {
                    path_builder.extendWithStraightTurningSegment(current_node->transition->path.front(), turning_straight_segment);
                } else {
                    path_builder.extendWithStraightTurningSegment(current_node->transition->path.back(), turning_straight_segment);
                }

                path_builder.insertCurveSegment(current_node);

                if(current_node->curve_forward) {
                    path_builder.extendAlongTargetSegment(current_node, turning_straight_segment);
                } else {
                    path_builder.extendAlongSourceSegment(current_node, turning_straight_segment);
                }
            }

        } else {
            // direction changed, there are four cases to check
            if(segment_forward) {
                if(current_node->curve_forward) {
                    // segment (S) forward + curve (C) forward + next segment (N) is backward
                    /*            v
                     *            v  N
                     * > > > > ,  v
                     *   S      '.v
                     *        C   v
                     *            *
                     *            * Extension after curve along target segment of C
                     *            *
                     */
                    path_builder.insertCurveSegment(current_node);
                    path_builder.extendAlongTargetSegment(current_node, turning_straight_segment);

                } else {
                    // segment (S) forward + curve (C) backward + next segment (N) is backward
                    /*            v
                     *            v  N
                     *            v,
                     *            v',  C
                     *            v  `'.,
                     * > > > > > >v> > > > *******
                     *   S                     Extension before curve along target segment of C
                     */
                    path_builder.extendAlongTargetSegment(current_node, turning_straight_segment);
                    path_builder.insertCurveSegment(current_node);
                }

            } else {
                if(current_node->curve_forward) {
                    // segment (S) backward + curve (C) forward + next segment (N) is forward
                    /*            ^
                     *            ^  S
                     * < < < < ,  ^
                     *   N      '.^
                     *        C   ^
                     *            *
                     *            * Extension before curve along source segment of C
                     *            *
                     */
                    path_builder.extendAlongSourceSegment(current_node, turning_straight_segment);
                    path_builder.insertCurveSegment(current_node);

                } else {
                    // segment (S) backward + curve (C) backward + next segment (N) is forward
                    /*            ^
                     *            ^  S
                     *            ^,
                     *            ^',  C
                     *            ^  `'.,
                     * < < < < < <^< < < < *******
                     *   N                     Extension after curve along source segment of C
                     */
                    path_builder.insertCurveSegment(current_node);
                    path_builder.extendAlongSourceSegment(current_node, turning_straight_segment);
                }
            }
        }

        segment_forward = next_segment_forward;
    }

    path_builder.insertTangentPoint(end_segment, end_pt);
}



std::string Search::signature(const Node *head) const
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
    std::string start_sym = isSegmentForward(start_segment, start_pt, end_point_on_start_segment) ? ">" : "<";

    return start_sym + res;
}
