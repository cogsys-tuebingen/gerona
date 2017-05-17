#include "search.h"

#include <cslibs_path_planning/common/CollisionGridMap2d.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <set>
#include <tf/tf.h>

#include "course_map.h"

#include "near_course_test.hpp"

namespace {
bool comp (const Node* lhs, const Node* rhs)
{
    return lhs->cost<rhs->cost;
}
}

Search::Search(const CourseMap& generator)
    : pnh_("~"),
      cost_calculator_(*this),
      generator_(generator)
{    
    pnh_.param("size/forward", size_forward, 0.4);
    pnh_.param("size/backward", size_backward, -0.6);
    pnh_.param("size/width", size_width, 0.5);

    pnh_.param("max_distance_for_direct_try", max_distance_for_direct_try, 7.0);
    pnh_.param("max_time_for_direct_try", max_time_for_direct_try, 1.0);
}


path_msgs::PathSequence Search::findPath(lib_path::SimpleGridMap2d * map,
                                         const path_msgs::PlanPathGoal &goal,
                                         const path_geom::PathPose& start_pose,
                                         const path_geom::PathPose& end_pose)
{
    map_info = map;

    world_frame_ = goal.goal.pose.header.frame_id;
    time_stamp_ = goal.goal.pose.header.stamp;

    double distance = (start_pose.pos_ - end_pose.pos_).norm();
    if(distance <= max_distance_for_direct_try) {
        auto direct = tryDirectPath(start_pose, end_pose);
        if(!direct.paths.empty()) {
            return direct;
        }
    }

    if(!findAppendices(start_pose, end_pose)) {
        return {};
    }


    if(start_segment == end_segment) {
        PathBuilder path_builder(*this);
        path_builder.addPath(start_appendix);

        path_builder.insertTangentPoint(start_segment, start_pt);
        path_builder.insertTangentPoint(end_segment, end_pt);

        path_builder.addPath(end_appendix);

        return path_builder;
    }

    return performDijkstraSearch();
}


path_msgs::PathSequence Search::tryDirectPath(const path_geom::PathPose& start, const path_geom::PathPose& end)
{
//    {
//        AStarPatsyReversed algo_forward;
//        algo_forward.setMap(map_info);
//        algo_forward.setTimeLimit(max_time_for_direct_try);
//        auto path = algo_forward.findPath(convertToMap(start), convertToMap(end));

//        if(!path.empty()) {
//            return convertToWorld(path);
//        }
//    }
    {
        AStarPatsyReversedTurning algo_turning;
        algo_turning.setMap(map_info);
        algo_turning.setTimeLimit(max_time_for_direct_try * 10);
        auto path = algo_turning.findPath(convertToMap(start), convertToMap(end));

        if(!path.empty()) {
            return convertToWorld(path);
        }
    }

    return {};
}

path_msgs::PathSequence Search::performDijkstraSearch()
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

                double curve_cost = cost_calculator_.calculateCurveCost(current_node);
                double straight_cost = cost_calculator_.calculateStraightCost(current_node,
                                                                      cost_calculator_.findStartPointOnSegment(current_node),
                                                                      cost_calculator_.findEndPointOnSegment(current_node, &next_transition));

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

    PathBuilder path_builder(*this);
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
            node->cost = cost_calculator_.calculateStraightCost(node, start_pt, end_point_on_segment);
            queue.insert(node);
        }
    }
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

bool Search::findAppendices(const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose)
{
    ROS_DEBUG_STREAM("searching appendices");

    start_appendix = findAppendix<AStarPatsyForward, AStarPatsyForwardTurning>(start_pose, "start");
    if(start_appendix.paths.empty()) {
        return false;
    }

    geometry_msgs::PoseStamped start_msg = start_appendix.paths.back().poses.back();
    path_geom::PathPose start(start_msg.pose.position.x, start_msg.pose.position.y,
                              tf::getYaw(start_msg.pose.orientation));
    start_segment = generator_.findClosestSegment(start, M_PI / 8, 0.5);
    start_pt = start_segment->line.nearestPointTo(start.pos_);
    if(!start_segment) {
        ROS_ERROR_STREAM("cannot find a path for start pose " << start.pos_);
        return false;
    }


    end_appendix = findAppendix<AStarPatsyReversed, AStarPatsyReversedTurning>(end_pose, "end");
    if(end_appendix.paths.empty()) {
        return false;
    }
//    std::reverse(end_appendix.begin(), end_appendix.end());

    geometry_msgs::PoseStamped end_msg = end_appendix.paths.front().poses.front();
    path_geom::PathPose end(end_msg.pose.position.x, end_msg.pose.position.y,
                              tf::getYaw(end_msg.pose.orientation));

    end_segment = generator_.findClosestSegment(end, M_PI / 8, 0.5);
    if(!end_segment) {
        ROS_ERROR_STREAM("cannot find a path for  end pose " << end.pos_);
        return false;
    }
    end_pt = end_segment->line.nearestPointTo(end.pos_);

    return true;
}

template <typename AlgorithmForward, typename AlgorithmFull>
path_msgs::PathSequence Search::findAppendix(const path_geom::PathPose& pose, const std::string& type)
{
    lib_path::Pose2d pose_map = convertToMap(pose);

    AlgorithmForward algo_forward;
    algo_forward.setMap(map_info);

    NearCourseTest<AlgorithmForward> goal_test_forward(generator_, algo_forward, map_info);
    auto path_start = algo_forward.findPath(pose_map, goal_test_forward, [](){});
    if(path_start.empty()) {
        ROS_WARN_STREAM("cannot connect to " << type << " without turning");
        AlgorithmFull algo_forward_turn;
        NearCourseTest<AlgorithmFull> goal_test_forward_turn(generator_, algo_forward_turn, map_info);
        algo_forward_turn.setMap(map_info);
        path_start = algo_forward_turn.findPath(pose_map, goal_test_forward_turn, [](){});
        if(path_start.empty()) {
            ROS_ERROR_STREAM("cannot connect to " << type);
            return {};
        }
    }

    return convertToWorld(path_start);
}




path_geom::PathPose Search::convertToWorld(const NodeT& node)
{
    double tmpx, tmpy;

    map_info->cell2pointSubPixel(node.x, node.y, tmpx, tmpy);

    return path_geom::PathPose(tmpx, tmpy, node.theta/* - tf::getYaw(map.info.origin.orientation)*/);
}

path_msgs::PathSequence Search::convertToWorld(const std::vector<NodeT>& path_raw)
{
    if(path_raw.empty()) {
        return {};
    }

    path_msgs::PathSequence path_out;
    path_out.paths.emplace_back();

    path_out.header.stamp = time_stamp_;
    path_out.header.frame_id = world_frame_;

    path_msgs::DirectionalPath* path = &path_out.paths.back();

    path->forward = true;

    if(path_raw.size() > 0) {
        for(const auto& next_map : path_raw) {
            geometry_msgs::PoseStamped pose;
            map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                         pose.pose.position.y);

            pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

            path->poses.push_back(pose);
        }
    }

    return path_out;
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




void Search::generatePathCandidate(Node* node)
{
    // finish the path -> connect to end point
    double additional_cost = cost_calculator_.calculateStraightCost(node, cost_calculator_.findStartPointOnSegment(node), end_pt);
    node->cost += additional_cost;

    ROS_DEBUG_STREAM("found candidate with signature " << cost_calculator_.signature(node) << " with cost " << node->cost);
    if(node->cost < min_cost) {
        min_cost = node->cost;
        best_path = path_msgs::PathSequence();

        std::deque<const Node*> transitions;
        Node* tmp = node;
        while(tmp) {
            transitions.push_front(tmp);
            if(tmp->prev) {
                tmp->prev->next = tmp;
            }
            tmp = tmp->prev;
        }

        PathBuilder path_builder(*this);
        generatePath(transitions, path_builder);
        best_path = path_builder;

        ROS_DEBUG_STREAM("best path has " << best_path.paths.size() << " paths");
        for(const auto& path : best_path.paths) {
            ROS_DEBUG_STREAM("* poses: " << path.poses.size() << ", direction: " << (bool) path.forward);
        }
    }
}

void Search::generatePath(const std::deque<const Node*>& path_transitions, PathBuilder& path_builder) const
{
    path_builder.insertTangentPoint(start_segment, start_pt);

    bool segment_forward = cost_calculator_.isStartSegmentForward(path_transitions.front());

    ROS_DEBUG_STREAM("generating path from " << path_transitions.size() << " transitions");
    for(std::size_t i = 0, n = path_transitions.size(); i < n; ++i) {
        const Node* current_node = path_transitions[i];

        double eff_len = cost_calculator_.calculateEffectiveLengthOfNextSegment(current_node);
        if(eff_len < std::numeric_limits<double>::epsilon()) {
            // the segment has no length -> only insert the transition curve
            path_builder.insertCurveSegment(current_node);
            continue;
        }

        bool next_segment_forward = cost_calculator_.isNextSegmentForward(current_node);

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
                    path_builder.extendWithStraightTurningSegment(current_node->transition->path.front(), cost_calculator_.turning_straight_segment);
                } else {
                    path_builder.extendWithStraightTurningSegment(current_node->transition->path.back(), cost_calculator_.turning_straight_segment);
                }

                path_builder.insertCurveSegment(current_node);

                if(current_node->curve_forward) {
                    path_builder.extendAlongTargetSegment(current_node, cost_calculator_.turning_straight_segment);
                } else {
                    path_builder.extendAlongSourceSegment(current_node, cost_calculator_.turning_straight_segment);
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
                    path_builder.extendAlongTargetSegment(current_node, cost_calculator_.turning_straight_segment);

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
                    path_builder.extendAlongTargetSegment(current_node, cost_calculator_.turning_straight_segment);
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
                    path_builder.extendAlongSourceSegment(current_node, cost_calculator_.turning_straight_segment);
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
                    path_builder.extendAlongSourceSegment(current_node, cost_calculator_.turning_straight_segment);
                }
            }
        }

        segment_forward = next_segment_forward;
    }

    path_builder.insertTangentPoint(end_segment, end_pt);
}
