#include "search.h"

#include <utils_path/geometry/intersector.h>
#include <utils_path/common/CollisionGridMap2d.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <set>
#include <tf/tf.h>

#include "course_generator.h"

namespace {
bool comp (const Node* lhs, const Node* rhs)
{
    return lhs->cost<rhs->cost;
}

template <typename Algorithm>
struct PathGoalTest
{
    PathGoalTest(CourseGenerator& parent, Algorithm& algo, const nav_msgs::OccupancyGrid& map, const lib_path::SimpleGridMap2d* map_info)
        : parent(parent),
          algo(algo), map(map), map_info(map_info),
          res(map.info.resolution),
          ox(map.info.origin.position.x),
          oy(map.info.origin.position.y),
          w(map.info.width),
          h(map.info.height),

          candidates(0)
    {
    }

    void reset()
    {
        candidates = 0;
    }

    bool terminate(const typename Algorithm::NodeT* node) const
    {
        double wx, wy;
        map_info->cell2pointSubPixel(node->x,node->y, wx, wy);

        const double min_necessary_dist_to_crossing = 0.0;

        path_geom::PathPose point(wx, wy, node->theta);
        const Segment* closest_segment = parent.findClosestSegment(point, M_PI / 8, 0.1);
        if(closest_segment) {
            double min_dist_to_crossing = std::numeric_limits<double>::infinity();
            for(const Transition& transition : closest_segment->forward_transitions) {
                double distance = (transition.intersection - point.pos_).norm();
                if(distance < min_dist_to_crossing)  {
                    min_dist_to_crossing = distance;
                }
            }
            for(const Transition& transition : closest_segment->backward_transitions) {
                double distance = (transition.intersection - point.pos_).norm();
                if(distance < min_dist_to_crossing)  {
                    min_dist_to_crossing = distance;
                }
            }

            if(min_dist_to_crossing > min_necessary_dist_to_crossing) {
                algo.addGoalCandidate(node, closest_segment->line.distanceTo(point.pos_));
                ++candidates;
            }

        }

        return candidates > 40;
    }

    const lib_path::Pose2d* getHeuristicGoal() const
    {
        return NULL;
    }

    CourseGenerator& parent;

    Algorithm& algo;
    const nav_msgs::OccupancyGrid& map;
    const lib_path::SimpleGridMap2d * map_info;

    double res;
    double ox;
    double oy;
    int w;
    int h;

    mutable int candidates;
};

}

Search::Search(CourseGenerator& generator)
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

    algo_forward.setMap(map_info.get());
    algo_reverse.setMap(map_info.get());
}

std::vector<path_geom::PathPose> Search::findPath(const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose)
{
    std::vector<path_geom::PathPose> res;

    nav_msgs::GetMap map_service;
    if(!map_service_client_.exists()) {
        map_service_client_.waitForExistence();
    }
    if(!map_service_client_.call(map_service)) {
        ROS_ERROR("map service lookup failed");
        return res;
    }

    const nav_msgs::OccupancyGrid& map = map_service.response.map;

    initMaps(map);

    ROS_WARN_STREAM("searching appendices");

    lib_path::Pose2d from_map = convertToMap(start_pose);
    lib_path::Pose2d to_map = convertToMap(end_pose);


    PathGoalTest<AStarPatsyForward> goal_test_forward(generator_, algo_forward, map, map_info.get());
    auto path_start = algo_forward.findPath(from_map, goal_test_forward, 0);
    if(path_start.empty()) {
        ROS_WARN("cannot connect to start without turning");
        PathGoalTest<AStarPatsyForwardTurning> goal_test_forward_turn(generator_, algo_forward_turn, map, map_info.get());
        algo_forward_turn.setMap(map_info.get());
        path_start = algo_forward_turn.findPath(from_map, goal_test_forward_turn, 0);
        if(path_start.empty()) {
            ROS_ERROR("cannot connect to start");
            return res;
        }
    }

    ROS_WARN_STREAM("start path: " << path_start.size());
    start = convertToWorld(path_start.back());

    PathGoalTest<AStarPatsyReversed> goal_test_reverse(generator_, algo_reverse, map, map_info.get());
    auto path_end = algo_reverse.findPath(to_map, goal_test_reverse, 0);

    if(path_end.empty()) {
        algo_reverse_turn.setMap(map_info.get());
        ROS_WARN("cannot connect to end without turning");
        PathGoalTest<AStarPatsyReversedTurning> goal_test_reverse_turn(generator_, algo_reverse_turn, map, map_info.get());
        path_end = algo_reverse_turn.findPath(to_map, goal_test_reverse_turn, 0);
        if(path_end.empty()) {
            ROS_ERROR("cannot connect to end");
            return res;
        }
    }

    std::reverse(path_end.begin(), path_end.end());
    ROS_WARN_STREAM("end path: " << path_end.size());
    end = convertToWorld(path_end.front());

    // find the closest segments and then perform bfs
    start_segment = generator_.findClosestSegment(start, M_PI / 8, 0.5);
    if(!start_segment) {
        ROS_ERROR_STREAM("cannot find a path for start pose " << start.pos_);
        return res;
    }

    end_segment = generator_.findClosestSegment(end, M_PI / 8, 0.5);
    if(!end_segment) {
        ROS_ERROR_STREAM("cannot find a path for end pose " << end.pos_);
        return res;
    }

    start_pt = start_segment->line.nearestPointTo(start.pos_);
    end_pt = end_segment->line.nearestPointTo(end.pos_);

    if(start_segment == end_segment) {
        Eigen::Vector2d start_delta = start_segment->line.endPoint() - start_segment->line.startPoint();
        double s_yaw = std::atan2(start_delta(1), start_delta(0));
        res.push_back(path_geom::PathPose(start_pt(0), start_pt(1), s_yaw));

        Eigen::Vector2d end_delta = end_segment->line.endPoint() - end_segment->line.startPoint();
        double e_yaw = std::atan2(end_delta(1), end_delta(0));
        res.push_back(path_geom::PathPose(end_pt(0), end_pt(1), e_yaw));

        return combine(path_start, res, path_end);
    }

    std::map<const Transition*, Node> nodes;
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

    std::set<Node*, bool(*)(const Node*, const Node*)> Q(&comp);


    Eigen::Vector2d start_segment_dir = start_segment->line.endPoint() - start_segment->line.startPoint();
    start_segment_dir /= start_segment_dir.norm();


    for(int i = 0; i <= 1; ++i) {
        const auto& transitions = i == 0 ? start_segment->forward_transitions : start_segment->backward_transitions;
        for(const Transition& next_transition : transitions) {

            Node* node = &nodes.at(&next_transition);

            // distance from start_pt to transition
            Eigen::Vector2d  end_point_on_segment = node->curve_forward ? next_transition.path.front() : next_transition.path.back();
            node->cost = calculateStraightCost(node, start_pt, end_point_on_segment);
            Q.insert(node);
        }
    }

    min_cost = std::numeric_limits<double>::infinity();

    while(!Q.empty()) {
        Node* current_node = *Q.begin();
        Q.erase(Q.begin());

        ROS_ASSERT(current_node->cost > 0);

        if(current_node->next_segment == end_segment) {
            generatePathCandidate(current_node);

            continue;
        }

        for(int i = 0; i <= 1; ++i) {
            const auto& transitions =  i == 0 ? current_node->next_segment->forward_transitions : current_node->next_segment->backward_transitions;
            for(const Transition& next_transition : transitions) {
                ROS_ASSERT(&next_transition != current_node->transition);
                ROS_ASSERT(current_node->next_segment != end_segment);

                Node* neighbor = &nodes.at(&next_transition);

                double curve_cost = calculateCurveCost(current_node);
                double straight_cost = calculateStraightCost(current_node, findStartPointOnNextSegment(current_node), findEndPointOnSegment(current_node, &next_transition));

                double new_cost = current_node->cost + curve_cost + straight_cost;

                ROS_ASSERT(new_cost != std::numeric_limits<double>::infinity());
                ROS_ASSERT(current_node->cost > 0);
                ROS_ASSERT(new_cost > 0);

                if(new_cost < neighbor->cost) {
                    neighbor->prev = current_node;
                    current_node->next = neighbor;

                    ROS_ASSERT(new_cost < neighbor->cost);
                    ROS_ASSERT(new_cost > 0);
                    neighbor->cost = new_cost;

                    if(Q.find(neighbor) != Q.end()) {
                        Q.erase(neighbor);
                    }
                    Q.insert(neighbor);
                }
            }
        }
    }


    return combine(path_start, best_path, path_end);
}

Eigen::Vector2d Search::findStartPointOnNextSegment(const Node* node) const
{
    if(node->next_segment == start_segment) {
        return start_segment->line.nearestPointTo(start.pos_);

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
        return end_segment->line.nearestPointTo(end.pos_);

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




bool Search::isPreviousSegmentForward(Node* node) const
{
    if(node->prev) {
        return isNextSegmentForward(node->prev);
    } else {
        return isSegmentForward(start_segment, start.pos_, findEndPointOnSegment(node, node->transition));
    }
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


template <typename NodeT>
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

template <typename PathT>
std::vector<path_geom::PathPose> Search::combine(const PathT& start,
                                                          const std::vector<path_geom::PathPose>& centre,
                                                          const PathT& end)
{
    if(start.empty() && end.empty()) {
        return centre;
    }

    std::vector<path_geom::PathPose> res;

    for(const auto& node : start) {
        res.push_back(convertToWorld(node));
    }

    res.insert(res.end(), centre.begin(), centre.end());

    for(const auto& node : end) {
        res.push_back(convertToWorld(node));
    }

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

        generatePath(transitions, best_path);
    }
}

void Search::generatePath(const std::deque<const Node*>& path_transitions, std::vector<path_geom::PathPose>& res) const
{
    Eigen::Vector2d first_pos = start_segment->line.nearestPointTo(start.pos_);
    const Node* first_node = path_transitions.front();

    Eigen::Vector2d first_delta = first_node->transition->path.front() - first_pos;
    double first_yaw = std::atan2(first_delta(1), first_delta(0));
    ROS_INFO("insert first point");
    res.push_back(path_geom::PathPose(first_pos(0), first_pos(1), first_yaw));

    Eigen::Vector2d  end_point_on_start_segment = path_transitions.front()->curve_forward ? path_transitions.front()->transition->path.front() : path_transitions.front()->transition->path.back();
    bool last_segment_forward = isSegmentForward(start_segment, start.pos_, end_point_on_start_segment);
    if(last_segment_forward) {
        ROS_INFO_STREAM("start segment is forward");
    } else {
        ROS_INFO_STREAM("start segment is backward");
    }


    ROS_INFO_STREAM("generating path from " << path_transitions.size() << " transitions");
    for(std::size_t i = 0, n = path_transitions.size(); i < n; ++i) {
        const Node* current_node = path_transitions[i];

        double eff_len = effectiveLengthOfNextSegment(current_node);
        if(eff_len < std::numeric_limits<double>::epsilon()) {
            ROS_WARN_STREAM("skipping segment of length " << eff_len);
            insertCurveSegment(res, current_node);
            continue;
        }

        bool next_segment_forward = isNextSegmentForward(current_node);
        if(next_segment_forward) {
            ROS_INFO_STREAM("segment is forward");
        } else {
            ROS_INFO_STREAM("segment is backward");
        }

        // insert turning point?
        if(next_segment_forward == last_segment_forward) {
            if(current_node->curve_forward == next_segment_forward) {
                ROS_INFO("no straight segment necessary");
                insertCurveSegment(res, current_node);

            } else {
                ROS_WARN("inserting straight segment at double turn");

                if(current_node->curve_forward) {
                    extendWithStraightTurningSegment(res, current_node->transition->path.front());
                } else {
                    extendWithStraightTurningSegment(res, current_node->transition->path.back());
                }

                insertCurveSegment(res, current_node);


                if(current_node->curve_forward) {
                    extendAlongTargetSegment(res, current_node);
                } else {
                    extendAlongSourceSegment(res, current_node);
                }
            }

        } else {
            if(last_segment_forward) {
                ROS_WARN("inserting straight segment at turning (forward)");
                if(current_node->curve_forward) {
                    insertCurveSegment(res, current_node);
                    extendAlongTargetSegment(res, current_node);
                } else {
                    extendAlongTargetSegment(res, current_node);
                    insertCurveSegment(res, current_node);
                }

            } else {
                ROS_WARN("inserting straight segment at turning (backward)");
                if(current_node->curve_forward) {
                    extendAlongSourceSegment(res, current_node);
                    insertCurveSegment(res, current_node);
                } else {
                    insertCurveSegment(res, current_node);
                    extendAlongSourceSegment(res, current_node);
                }
            }
        }

        last_segment_forward = next_segment_forward;
    }

    Eigen::Vector2d last_pos = end_segment->line.nearestPointTo(end.pos_);
    const Node* last_node = path_transitions.back();

    Eigen::Vector2d last_delta = last_pos - last_node->transition->path.back();
    double last_yaw = std::atan2(last_delta(1), last_delta(0));
    ROS_INFO("insert end point");
    res.push_back(path_geom::PathPose(last_pos(0), last_pos(1), last_yaw));
}

void Search::extendAlongTargetSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const
{
    ROS_INFO("extend along next");
    const Transition& current_transition = *current_node->transition;

    double c_yaw = 0;
    Eigen::Vector2d  pt = current_transition.path.back();

    Eigen::Vector2d ep = current_transition.target->line.endPoint();
    Eigen::Vector2d sp = current_transition.target->line.startPoint();
    Eigen::Vector2d delta = ep - sp;

    c_yaw = std::atan2(delta(1), delta(0));

    Eigen::Vector2d offset(turning_straight_segment, 0.0);
    Eigen::Matrix2d rot;
    rot << std::cos(c_yaw), -std::sin(c_yaw), std::sin(c_yaw), std::cos(c_yaw);

    pt += rot * offset;

    res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
}

void Search::extendAlongSourceSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const
{
    ROS_INFO("extend along current");
    const Transition& current_transition = *current_node->transition;

    double c_yaw = 0;
    Eigen::Vector2d pt = current_transition.path.front();


    Eigen::Vector2d ep = current_transition.source->line.endPoint();
    Eigen::Vector2d sp = current_transition.source->line.startPoint();
    Eigen::Vector2d delta = ep - sp;

    c_yaw = std::atan2(delta(1), delta(0)) + M_PI;

    Eigen::Vector2d offset(turning_straight_segment, 0.0);
    Eigen::Matrix2d rot;
    rot << std::cos(c_yaw), -std::sin(c_yaw), std::sin(c_yaw), std::cos(c_yaw);

    pt += rot * offset;

    res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
}

void Search::extendWithStraightTurningSegment(std::vector<path_geom::PathPose>& res, const Eigen::Vector2d& pt) const
{
    ROS_INFO("extend straight");
    Eigen::Vector2d prev_pt = res.back().pos_;

    Eigen::Vector2d dir = (pt - prev_pt);
    Eigen::Vector2d offset = dir / dir.norm() * turning_straight_segment;
    Eigen::Vector2d pos = pt + offset;

    res.push_back(path_geom::PathPose(pos(0), pos(1), std::atan2(dir(1), dir(0))));
}

void Search::insertCurveSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const
{
    const Transition& current_transition = *current_node->transition;

    if(current_node->curve_forward) {
        ROS_INFO("insert curve: curve is forward");
        for(std::size_t j = 1, m = current_transition.path.size(); j < m; ++j) {
            const Eigen::Vector2d& pt = current_transition.path.at(j);
            const Eigen::Vector2d& prev_pt = current_transition.path.at(j-1);
            Eigen::Vector2d delta = pt - prev_pt;
            double c_yaw = std::atan2(delta(1), delta(0));
            res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
        }

    } else {
        ROS_INFO("insert curve: curve is backward");
        for(int m = current_transition.path.size(), j = m-2; j >= 0; --j) {
            const Eigen::Vector2d& pt = current_transition.path.at(j);
            const Eigen::Vector2d& next_pt = current_transition.path.at(j+1);
            Eigen::Vector2d delta = pt - next_pt;
            double c_yaw = std::atan2(delta(1), delta(0));
            res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
        }
    }
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
    std::string start_sym = isSegmentForward(start_segment, start.pos_, end_point_on_start_segment) ? ">" : "<";

    return start_sym + res;
}
