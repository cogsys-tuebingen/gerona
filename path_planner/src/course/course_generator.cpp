#include "course_generator.h"
#include <utils_path/geometry/intersector.h>
#include <utils_path/common/CollisionGridMap2d.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <set>
#include <tf/tf.h>

double CourseGenerator::Transition::arc_length() const
{
    return std::abs(dtheta * r);
}


CourseGenerator::CourseGenerator(ros::NodeHandle &nh)
    : nh_(nh), pnh_("~")
{
    pub_viz_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, true);

    pnh_.param("course/radius", curve_radius, 1.0);
    pnh_.param("course/penalty/backwards", backward_penalty_factor, 2.5);
    pnh_.param("course/penalty/turn", turning_penalty, 5.0);

    pnh_.param("course/turning/straight", turning_straight_segment, 0.7);

    pnh_.param("size/forward", size_forward, 0.4);
    pnh_.param("size/backward", size_backward, -0.6);
    pnh_.param("size/width", size_width, 0.5);

    std::string map_service = "/static_map";
    pnh_.param("map_service",map_service, map_service);
    map_service_client_ = pnh_.serviceClient<nav_msgs::GetMap> (map_service);
}

CourseGenerator::~CourseGenerator()
{

}

Eigen::Vector2d CourseGenerator::readPoint(const XmlRpc::XmlRpcValue& value, int index)
{
    XmlRpc::XmlRpcValue pt = value[index];
    if(pt.size() != 2) {
        ROS_FATAL_STREAM("point " << pt << " has an invalid format, should be [x, y]");
        std::abort();
    }

    double x = pt[0];
    double y = pt[1];
    return Eigen::Vector2d (x, y);
}

namespace {
bool comp (const CourseGenerator::Node* lhs, const CourseGenerator::Node* rhs)
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
        const CourseGenerator::Segment* closest_segment = parent.findClosestSegment(point, M_PI / 8, 0.1);
        if(closest_segment) {
            double min_dist_to_crossing = std::numeric_limits<double>::infinity();
            for(const CourseGenerator::Transition& transition : closest_segment->forward_transitions) {
                double distance = (transition.intersection - point.pos_).norm();
                if(distance < min_dist_to_crossing)  {
                    min_dist_to_crossing = distance;
                }
            }
            for(const CourseGenerator::Transition& transition : closest_segment->backward_transitions) {
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

void CourseGenerator::initMaps(const nav_msgs::OccupancyGrid& map)
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

std::vector<path_geom::PathPose> CourseGenerator::findPath(const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose)
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


    PathGoalTest<AStarPatsyForward> goal_test_forward(*this, algo_forward, map, map_info.get());
    auto path_start = algo_forward.findPath(from_map, goal_test_forward, 0);
    if(path_start.empty()) {
        ROS_WARN("cannot connect to start without turning");
        PathGoalTest<AStarPatsyForwardTurning> goal_test_forward_turn(*this, algo_forward_turn, map, map_info.get());
        algo_forward_turn.setMap(map_info.get());
        path_start = algo_forward_turn.findPath(from_map, goal_test_forward_turn, 0);
        if(path_start.empty()) {
            ROS_ERROR("cannot connect to start");
            return res;
        }
    }

    ROS_WARN_STREAM("start path: " << path_start.size());
    start = convertToWorld(path_start.back());

    PathGoalTest<AStarPatsyReversed> goal_test_reverse(*this, algo_reverse, map, map_info.get());
    auto path_end = algo_reverse.findPath(to_map, goal_test_reverse, 0);

    if(path_end.empty()) {
        algo_reverse_turn.setMap(map_info.get());
        ROS_WARN("cannot connect to end without turning");
        PathGoalTest<AStarPatsyReversedTurning> goal_test_reverse_turn(*this, algo_reverse_turn, map, map_info.get());
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
    start_segment = findClosestSegment(start, M_PI / 8, 0.5);
    if(!start_segment) {
        ROS_ERROR_STREAM("cannot find a path for start pose " << start.pos_);
        return res;
    }

    end_segment = findClosestSegment(end, M_PI / 8, 0.5);
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
    for(const Segment& s : segments_) {
        for(const Transition& t : s.forward_transitions) {
            nodes[&t].transition = &t;
            nodes[&t].curve_forward = true;
            nodes[&t].associated_segment = t.target;
        }
        for(const Transition& t : s.backward_transitions) {
            nodes[&t].transition = &t;
            nodes[&t].curve_forward = false;
            nodes[&t].associated_segment = t.source;
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

        if(current_node->associated_segment == end_segment) {
            generatePathCandidate(current_node);

            continue;
        }

        for(int i = 0; i <= 1; ++i) {
            const auto& transitions =  i == 0 ? current_node->associated_segment->forward_transitions : current_node->associated_segment->backward_transitions;
            for(const Transition& next_transition : transitions) {
                ROS_ASSERT(&next_transition != current_node->transition);
                ROS_ASSERT(current_node->associated_segment != end_segment);

                Node* neighbor = &nodes.at(&next_transition);

                double curve_cost = calculateCurveCost(current_node);
                double straight_cost = calculateStraightCost(current_node, findStartPointOnSegment(current_node), findEndPointOnSegment(current_node, &next_transition));

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

Eigen::Vector2d CourseGenerator::findStartPointOnSegment(const Node* node) const
{
    if(node->associated_segment == start_segment) {
        return start_segment->line.nearestPointTo(start.pos_);

    } else {
        return findStartPointOnSegment(node, node->transition);
    }
}

Eigen::Vector2d CourseGenerator::findStartPointOnSegment(const Node* node, const Transition* transition) const
{
    if(node->curve_forward) {
        return transition->path.back();
    } else {
        return transition->path.front();
    }
}


Eigen::Vector2d CourseGenerator::findEndPointOnSegment(const Node* node) const
{
    if(node->associated_segment == end_segment) {
        return end_segment->line.nearestPointTo(end.pos_);

    } else if(!node->next) {
        return node->curve_forward ? node->associated_segment->line.endPoint() : node->associated_segment->line.startPoint();

    } else  {
        const Node* next_node = node->next;
        return findEndPointOnSegment(next_node, next_node->transition);
    }

//    return findEndPointOnSegment(node, node->transition);
}


Eigen::Vector2d CourseGenerator::findEndPointOnSegment(const Node* node, const Transition* transition) const
{
    if(node->curve_forward) {
        return transition->path.front();
    } else {
        return transition->path.back();
    }
}




bool CourseGenerator::isPreviousSegmentForward(Node* current_node) const
{
    if(current_node->prev) {
        return isAssociatedSegmentForward(current_node->prev);
    } else {
        return isSegmentForward(start_segment, start.pos_, findEndPointOnSegment(current_node, current_node->transition));
    }
}


bool CourseGenerator::isAssociatedSegmentForward(const CourseGenerator::Node* node) const
{
    return isSegmentForward(node->associated_segment, findStartPointOnSegment(node), findEndPointOnSegment(node));
}
double CourseGenerator::effectiveLengthOfAssociatedSegment(const CourseGenerator::Node* node) const
{
    return (findStartPointOnSegment(node) - findEndPointOnSegment(node)).norm();
}



bool CourseGenerator::isSegmentForward(const CourseGenerator::Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target) const
{
    Eigen::Vector2d segment_dir = segment->line.endPoint() - segment->line.startPoint();
    Eigen::Vector2d move_dir = target - pos;
    if(move_dir.norm() < 0.1) {
        ROS_WARN_STREAM("effective segment size is small: " << move_dir.norm());
    }
    return segment_dir.dot(move_dir) >= 0.0;
}


template <typename NodeT>
path_geom::PathPose CourseGenerator::convertToWorld(const NodeT& node)
{
    double tmpx, tmpy;

    map_info->cell2pointSubPixel(node.x, node.y, tmpx, tmpy);

    return path_geom::PathPose(tmpx, tmpy, node.theta/* - tf::getYaw(map.info.origin.orientation)*/);
}

lib_path::Pose2d  CourseGenerator::convertToMap(const path_geom::PathPose& pt)
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
std::vector<path_geom::PathPose> CourseGenerator::combine(const PathT& start,
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

double  CourseGenerator::calculateStraightCost(Node* node, const Eigen::Vector2d& start_point_on_segment, const Eigen::Vector2d& end_point_on_segment) const
{
    double cost = 0.0;
    bool segment_forward = isSegmentForward(node->associated_segment, start_point_on_segment, end_point_on_segment);
    double distance_to_end = (end_point_on_segment - start_point_on_segment).norm();
    if(segment_forward) {
        cost += distance_to_end;
    } else {
        cost += backward_penalty_factor * distance_to_end;
    }

    bool prev_segment_forward = isPreviousSegmentForward(node);
    if(prev_segment_forward != segment_forward) {
        cost += turning_straight_segment;
        cost += turning_penalty;
    }

    return cost;
}

double CourseGenerator::calculateCurveCost(Node *node) const
{
    if(node->curve_forward) {
        return node->transition->arc_length();
    } else {
        return backward_penalty_factor * node->transition->arc_length();
    }
}

void CourseGenerator::generatePathCandidate(Node* node)
{
    // finish the path -> connect to end point
    double additional_cost = calculateStraightCost(node, findStartPointOnSegment(node), end_pt);
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

void CourseGenerator::generatePath(const std::deque<const Node*>& path_transitions, std::vector<path_geom::PathPose>& res) const
{
    Eigen::Vector2d first_pos = start_segment->line.nearestPointTo(start.pos_);
    const Node* first_node = path_transitions.front();

    Eigen::Vector2d first_delta = first_node->transition->path.front() - first_pos;
    double first_yaw = std::atan2(first_delta(1), first_delta(0));
    ROS_INFO("insert first point");
    res.push_back(path_geom::PathPose(first_pos(0), first_pos(1), first_yaw));

    Eigen::Vector2d  end_point_on_associated_segment = path_transitions.front()->curve_forward ? path_transitions.front()->transition->path.front() : path_transitions.front()->transition->path.back();
    bool last_segment_forward = isSegmentForward(start_segment, start.pos_, end_point_on_associated_segment);
    if(last_segment_forward) {
        ROS_INFO_STREAM("start segment is forward");
    } else {
        ROS_INFO_STREAM("start segment is backward");
    }


    ROS_INFO_STREAM("generating path from " << path_transitions.size() << " transitions");
    for(std::size_t i = 0, n = path_transitions.size(); i < n; ++i) {
        const Node* current_node = path_transitions[i];

        double eff_len = effectiveLengthOfAssociatedSegment(current_node);
        if(eff_len < std::numeric_limits<double>::epsilon()) {
            ROS_WARN_STREAM("skipping segment of length " << eff_len);
            insertCurveSegment(res, current_node);
            continue;
        }

        bool segment_forward = isAssociatedSegmentForward(current_node);
        if(segment_forward) {
            ROS_INFO_STREAM("segment is forward");
        } else {
            ROS_INFO_STREAM("segment is backward");
        }

        // insert turning point?
        if(segment_forward == last_segment_forward) {
            if(current_node->curve_forward == segment_forward) {
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

        last_segment_forward = segment_forward;
    }

    Eigen::Vector2d last_pos = end_segment->line.nearestPointTo(end.pos_);
    const Node* last_node = path_transitions.back();

    Eigen::Vector2d last_delta = last_pos - last_node->transition->path.back();
    double last_yaw = std::atan2(last_delta(1), last_delta(0));
    ROS_INFO("insert end point");
    res.push_back(path_geom::PathPose(last_pos(0), last_pos(1), last_yaw));
}

void CourseGenerator::extendAlongTargetSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const
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

void CourseGenerator::extendAlongSourceSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const
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

void CourseGenerator::extendWithStraightTurningSegment(std::vector<path_geom::PathPose>& res, const Eigen::Vector2d& pt) const
{
    ROS_INFO("extend straight");
    Eigen::Vector2d prev_pt = res.back().pos_;

    Eigen::Vector2d dir = (pt - prev_pt);
    Eigen::Vector2d offset = dir / dir.norm() * turning_straight_segment;
    Eigen::Vector2d pos = pt + offset;

    res.push_back(path_geom::PathPose(pos(0), pos(1), std::atan2(dir(1), dir(0))));
}

void CourseGenerator::insertCurveSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const
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

const CourseGenerator::Segment* CourseGenerator::findClosestSegment(const path_geom::PathPose &pose, double yaw_tolerance, double max_dist) const
{
    Eigen::Vector2d pt = pose.pos_;
    double yaw = pose.theta_;

    double best_dist = max_dist + std::numeric_limits<double>::epsilon();
    const Segment* best_segment = nullptr;

    for(const Segment& segment : segments_) {
        Eigen::Vector2d delta = segment.line.endPoint() - segment.line.startPoint();
        double s_yaw = std::atan2(delta(1), delta(0));

        if(std::abs(MathHelper::NormalizeAngle(yaw - s_yaw)) > yaw_tolerance) {
            continue;
        }

        Eigen::Vector2d nearest = segment.line.nearestPointTo(pt);
        double dist = (nearest - pt).norm();
        if(dist < best_dist) {
            best_dist = dist;
            best_segment = &segment;
        }
    }

    return best_segment;
}

void CourseGenerator::createMap(const XmlRpc::XmlRpcValue &map_segment_array)
{
    for(int i =0; i < map_segment_array.size(); i++) {
        XmlRpc::XmlRpcValue segment = map_segment_array[i];

        if(segment.size() != 2) {
            ROS_FATAL_STREAM("segment " << segment << " has an invalid format, should be [start, end]");
            std::abort();
        }

        Eigen::Vector2d start = readPoint(segment, 0);
        Eigen::Vector2d end = readPoint(segment, 1);

        path_geom::Line line(start, end);

        Segment s(line);

        segments_.emplace_back(s);
    }

    for(std::size_t i = 0; i < segments_.size(); ++i) {
        for(std::size_t j = 0; j < segments_.size(); ++j) {
            if(i == j) {
                continue;
            }

            // check for intersection si to sj
            Segment &si = segments_.at(i);
            Segment &sj = segments_.at(j);

            double eps = 0.0005;

            Eigen::Vector2d intersection;
            if(path_geom::Intersector::intersect(si.line, sj.line, intersection, eps)) {
                // only forward motion is allowd
                //  -> if the intersection is at the start of the source segment -> ignore
                if((intersection - si.line.startPoint()).norm() < eps) {
                    continue;
                }
                //  -> if the intersection is at the end of the target segment -> ignore
                if((intersection - sj.line.endPoint()).norm() < eps) {
                    continue;
                }

                intersections_.push_back(intersection);

                Eigen::Vector2d a = si.line.endPoint() - si.line.startPoint();
                Eigen::Vector2d b = sj.line.endPoint() - sj.line.startPoint();

                Eigen::Vector2d an = a / a.norm();
                Eigen::Vector2d bn = b / b.norm();

                double opening_angle = std::atan2(bn(1), bn(0)) - std::atan2(an(1), an(0));
                opening_angle = MathHelper::NormalizeAngle(opening_angle);
                double phi = 0;
                if(opening_angle >= 0.0) {
                    phi = (M_PI - opening_angle) / 2.;
                } else {
                    phi = (-M_PI - opening_angle) / 2.;
                }

                Eigen::Matrix2d rot_to_icr;
                rot_to_icr << std::cos(phi), - std::sin(phi), std::sin(phi), std::cos(phi);

                Eigen::Vector2d to_icr = rot_to_icr * bn;
                to_icr *= curve_radius / (std::sin(std::abs(phi)) * to_icr.norm());

                Eigen::Vector2d icr = intersection + to_icr;

                Transition t;
                t.source = &si;
                t.target = &sj;
                t.icr = icr;
                t.intersection = intersection;
                t.r = curve_radius;


                Eigen::Vector2d s = si.line.projectPoint(t.icr);
                Eigen::Vector2d e = sj.line.projectPoint(t.icr);

                Eigen::Vector2d ds = s - t.icr;
                Eigen::Vector2d de = e - t.icr;

                double start_angle = std::atan2(ds(1), ds(0));
                double end_angle = std::atan2(de(1), de(0));

                t.dtheta = MathHelper::NormalizeAngle(end_angle - start_angle);

                double desired_step = M_PI / 32.0;

                int segments = std::abs((int) std::round(t.dtheta / desired_step));


                if(segments <= 1) {
                    t.path.push_back(s);
                    t.path.push_back(e);

                } else {
                    double step_size = t.dtheta / segments;

                    Eigen::Matrix2d delta;
                    delta << std::cos(step_size), -std::sin(step_size), std::sin(step_size), std::cos(step_size);

                    Eigen::Vector2d offset = ds;

                    for(int step = 0; step < segments; ++step) {
                        t.path.push_back(icr + offset);

                        offset = delta * offset;
                    }
                }

                si.forward_transitions.emplace_back(t);
                sj.backward_transitions.emplace_back(t);
            }
        }
    }
}

void CourseGenerator::publishMarkers() const
{
    visualization_msgs::MarkerArray array;

    addLines(array);
    addIntersections(array);
    addTransitions(array);


    pub_viz_.publish(array);
}

bool CourseGenerator::hasSegments() const
{
    return !segments_.empty();
}

void CourseGenerator::addLines(visualization_msgs::MarkerArray& array) const
{
    visualization_msgs::Marker templ;
    templ.type = visualization_msgs::Marker::ARROW;
    templ.id = 0;
    templ.action = visualization_msgs::Marker::ADD;
    templ.color.r = 0;
    templ.color.g = 0;
    templ.color.b = 1;
    templ.color.a = 1;
    templ.header.frame_id = "/map";
    templ.header.stamp = ros::Time(0);
    templ.ns = "course/map/lines";
    templ.scale.x = 0.05;
    templ.scale.y = 0.1;

    double desired_arrow_length = 1.0;

    for(const Segment& segment : segments_) {
        const path_geom::Line& line = segment.line;

        geometry_msgs::Point start, end;

        Eigen::Vector2d s = line.startPoint();
        Eigen::Vector2d e = line.endPoint();

        Eigen::Vector2d delta = (e - s);

        double length = delta.norm();
        int segments = std::round(length / desired_arrow_length);
        double arrow_length = length / segments;
        delta *= arrow_length / length;

        Eigen::Vector2d pt = s;

        for(int i = 0; i < segments; ++i) {
            Eigen::Vector2d next_pt = pt + delta;
            start.x = pt(0);
            start.y = pt(1);
            end.x = next_pt(0);
            end.y = next_pt(1);

            templ.points.clear();
            templ.points.push_back(start);
            templ.points.push_back(end);

            array.markers.push_back(templ);
            ++templ.id;

            pt = next_pt;
        }

    }
}

void CourseGenerator::addTransitions(visualization_msgs::MarkerArray& array) const
{
    visualization_msgs::Marker icr;
    icr.type = visualization_msgs::Marker::ARROW;
    icr.id = 0;
    icr.action = visualization_msgs::Marker::ADD;
    icr.color.r = 1;
    icr.color.g = 0;
    icr.color.b = 0;
    icr.color.a = 1;
    icr.header.frame_id = "/map";
    icr.header.stamp = ros::Time(0);
    icr.ns = "course/map/icr";
    icr.scale.x = 0.025;
    icr.scale.y = 0.05;


    visualization_msgs::Marker curve;
    curve.type = visualization_msgs::Marker::ARROW;
    curve.id = 0;
    curve.action = visualization_msgs::Marker::ADD;
    curve.color.r = 0;
    curve.color.g = 1;
    curve.color.b = 0;
    curve.color.a = 1;
    curve.header.frame_id = "/map";
    curve.header.stamp = ros::Time(0);
    curve.ns = "course/map/curves";
    curve.scale.x = 0.05;
    curve.scale.y = 0.1;


    for(const Segment& segment : segments_) {
        for(int i = 0; i <= 1; ++i) {
            const auto& transitions =  i == 0 ? segment.forward_transitions : segment.backward_transitions;
            for(const Transition& transition : transitions) {
                // icr
                geometry_msgs::Point start, end;
                Eigen::Vector2d delta = transition.icr - transition.intersection;
                Eigen::Vector2d e = transition.intersection + delta * (delta.norm() - transition.r) / delta.norm();

                start.x = transition.intersection(0);
                start.y = transition.intersection(1);
                end.x = e(0);
                end.y = e(1);

                icr.points.clear();
                icr.points.push_back(start);
                icr.points.push_back(end);

                array.markers.push_back(icr);
                ++icr.id;


                // curve
                for(int i = 0; i < ((int)transition.path.size()) - 1; ++i) {
                    Eigen::Vector2d pt = transition.path.at(i);
                    Eigen::Vector2d next_pt = transition.path.at(i+1);
                    start.x = pt(0);
                    start.y = pt(1);
                    end.x = next_pt(0);
                    end.y = next_pt(1);

                    curve.points.clear();
                    curve.points.push_back(start);
                    curve.points.push_back(end);

                    array.markers.push_back(curve);
                    ++curve.id;
                }
            }
        }
    }
}

void CourseGenerator::addIntersections(visualization_msgs::MarkerArray& array) const
{
    visualization_msgs::Marker templ;
    templ.type = visualization_msgs::Marker::CYLINDER;
    templ.id = 0;
    templ.action = visualization_msgs::Marker::ADD;
    templ.color.r = 1;
    templ.color.g = 0;
    templ.color.b = 1;
    templ.color.a = 0.25;
    templ.header.frame_id = "/map";
    templ.header.stamp = ros::Time(0);
    templ.ns = "course/map/intersections";
    templ.scale.x = 0.5;
    templ.scale.y = 0.5;
    templ.scale.z = 0.5;

    for(const Eigen::Vector2d& intersection : intersections_) {
        templ.pose.position.x = intersection(0);
        templ.pose.position.y = intersection(1);

        array.markers.push_back(templ);

        ++templ.id;
    }
}

std::string CourseGenerator::signature(const Node *head) const
{
    std::string res = "";

    const Node* first = head;
    const Node* tmp = head;
    while(tmp) {
        if(tmp->associated_segment) {
            std::string dir = isAssociatedSegmentForward(tmp) ? ">" : "<";
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
