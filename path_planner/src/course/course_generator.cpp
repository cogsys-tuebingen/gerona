#include "course_generator.h"
#include <utils_path/geometry/intersector.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

#include <set>

CourseGenerator::CourseGenerator(ros::NodeHandle &nh)
    : nh_(nh), pnh_("~")
{
    pub_viz_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, true);

    pnh_.param("course/radius", curve_radius, 1.0);
    pnh_.param("course/penalty/backwards", backward_penalty_factor, 2.5);
    pnh_.param("course/penalty/turn", turning_penalty, 5.0);
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

bool isSegmentForward(const CourseGenerator::Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target)
{
    Eigen::Vector2d segment_dir = segment->line.endPoint() - segment->line.startPoint();
    Eigen::Vector2d move_dir = target - pos;
    return segment_dir.dot(move_dir) > 0.0;
}
}

std::vector<path_geom::PathPose> CourseGenerator::findPath(const path_geom::PathPose& start, const path_geom::PathPose& end) const
{
    std::vector<path_geom::PathPose> res;

    // find the closest segments and then perform bfs
    const Segment* start_segment = findClosestSegment(start, M_PI / 8, 0.5);
    if(!start_segment) {
        ROS_ERROR_STREAM("cannot find a path for start pose " << start.pos_);
        return res;
    }

    const Segment* end_segment = findClosestSegment(end, M_PI / 8, 0.5);
    if(!end_segment) {
        ROS_ERROR_STREAM("cannot find a path for end pose " << end.pos_);
        return res;
    }

    if(start_segment == end_segment) {
        Eigen::Vector2d start_pt = start_segment->line.nearestPointTo(start.pos_);
        Eigen::Vector2d start_delta = start_segment->line.endPoint() - start_segment->line.startPoint();
        double s_yaw = std::atan2(start_delta(1), start_delta(0));
        res.push_back(path_geom::PathPose(start_pt(0), start_pt(1), s_yaw));

        Eigen::Vector2d end_pt = end_segment->line.nearestPointTo(end.pos_);
        Eigen::Vector2d end_delta = end_segment->line.endPoint() - end_segment->line.startPoint();
        double e_yaw = std::atan2(end_delta(1), end_delta(0));
        res.push_back(path_geom::PathPose(end_pt(0), end_pt(1), e_yaw));

        return res;
    }

    //    res.push_back(start);

    std::map<const Transition*, Node> nodes;
    for(const Segment& s : segments_) {
        for(const Transition& t : s.transitions) {
            nodes[&t].transition = &t;
        }
    }

    std::set<Node*, bool(*)(const Node*, const Node*)> Q(&comp);
    std::set<Node*, bool(*)(const Node*, const Node*)> candidates(&comp);

    Eigen::Vector2d start_pt = start_segment->line.nearestPointTo(start.pos_);
    Eigen::Vector2d end_pt = end_segment->line.nearestPointTo(end.pos_);

    Eigen::Vector2d start_segment_dir = start_segment->line.endPoint() - start_segment->line.startPoint();
    start_segment_dir /= start_segment_dir.norm();


    for(const Transition& t : start_segment->transitions) {
        Node& node = nodes.at(&t);
        bool curve_forward = std::abs(start_segment->line.distanceTo(t.path.front())) < std::abs(start_segment->line.distanceTo(t.path.back()));

        // distance from start_pt to transition
        Eigen::Vector2d  end_point_on_associated_segment = curve_forward ? t.path.front() : t.path.back();
        {
            Eigen::Vector2d  other_possibility = !curve_forward ? t.path.front() : t.path.back();
            ROS_ASSERT(start_segment->line.distanceTo(end_point_on_associated_segment) < start_segment->line.distanceTo(other_possibility));
        }

        node.curve_forward = curve_forward;

        bool segment_forward = isSegmentForward(start_segment, start_pt, end_point_on_associated_segment);

        double distance = (start_pt - end_point_on_associated_segment).norm();

        if(segment_forward) {
            node.cost = distance;
            node.distance_forward += distance;
            node.history.push_back(0);
            node.history.push_back(distance);
            node.history.push_back(0);
        } else {
            node.cost = backward_penalty_factor * distance;
            node.distance_backward += distance;
            node.history.push_back(0);
            node.history.push_back(-distance);
            node.history.push_back(0);
        }

        ROS_ASSERT(node.cost > 0);

        Q.insert(&node);
    }

    while(!Q.empty()) {
        Node* current_node = *Q.begin();
        Q.erase(Q.begin());

        ROS_ASSERT(current_node->cost > 0);

        const Transition& current_transition = *current_node->transition;

        double current_arc_length = std::abs(current_transition.dtheta * current_transition.r);


        const Segment* associated_segment = current_node->curve_forward ? current_transition.target : current_transition.source;

        Eigen::Vector2d start_point_on_segment;
        if(associated_segment == start_segment) {
            start_point_on_segment = start_pt;
        } else {
            start_point_on_segment = current_node->curve_forward ? current_transition.path.back() : current_transition.path.front();
            {
                Eigen::Vector2d  other_possibility = !current_node->curve_forward ? current_transition.path.back() : current_transition.path.front();
                ROS_ASSERT(associated_segment->line.distanceTo(start_point_on_segment) < associated_segment->line.distanceTo(other_possibility));
            }
        }

        if(associated_segment == end_segment) {
            bool segment_forward = isSegmentForward(associated_segment, start_point_on_segment, end_pt);

            double distance_to_end = (end_pt - start_point_on_segment).norm();
            if(segment_forward) {
                current_node->cost += distance_to_end;
                current_node->distance_forward += distance_to_end;
                current_node->history.push_back(distance_to_end);
            } else {
                current_node->cost += backward_penalty_factor * distance_to_end;
                current_node->distance_backward += distance_to_end;
                current_node->history.push_back(-distance_to_end);
            }

            candidates.insert(current_node);
            continue;
        }

        for(const Transition& next_transition : associated_segment->transitions) {
            if(&next_transition == &current_transition) {
                continue;
            }
            bool curve_forward = std::abs(associated_segment->line.distanceTo(next_transition.path.front())) < std::abs(associated_segment->line.distanceTo(next_transition.path.back()));

            ROS_ASSERT(associated_segment != end_segment);
            Eigen::Vector2d  end_point_on_associated_segment = curve_forward ? next_transition.path.front() : next_transition.path.back();
            {
                Eigen::Vector2d  other_possibility = !curve_forward ? next_transition.path.front() : next_transition.path.back();
                ROS_ASSERT(associated_segment->line.distanceTo(end_point_on_associated_segment) < associated_segment->line.distanceTo(other_possibility));
            }

            bool segment_forward = isSegmentForward(associated_segment, start_point_on_segment, end_point_on_associated_segment);



            double distance_to_end = (end_point_on_associated_segment - start_point_on_segment).norm();

            double new_cost = current_node->cost;
            if(curve_forward) {
                new_cost += current_arc_length;
            } else {
                new_cost += backward_penalty_factor * current_arc_length;
            }

            if(segment_forward) {
                new_cost += distance_to_end;
            } else {
                new_cost += backward_penalty_factor * distance_to_end;
            }

            if(curve_forward != segment_forward) {
                new_cost += turning_penalty;
            }

            ROS_ASSERT(new_cost != std::numeric_limits<double>::infinity());
            ROS_ASSERT(current_node->cost > 0);
            ROS_ASSERT(new_cost > 0);

            Node* next_transition_node = &nodes.at(&next_transition);
            if(new_cost < next_transition_node->cost) {
                next_transition_node->prev = current_node;
                ROS_ASSERT(new_cost < next_transition_node->cost);
                ROS_ASSERT(new_cost > 0);
                next_transition_node->cost = new_cost;
                next_transition_node->curve_forward = curve_forward;
                if(Q.find(next_transition_node) != Q.end()) {
                    Q.erase(next_transition_node);
                }

                next_transition_node->distance_forward = current_node->distance_forward;
                next_transition_node->distance_backward = current_node->distance_backward;

                next_transition_node->history = current_node->history;
                if(curve_forward) {
                    next_transition_node->distance_forward += current_arc_length;
                    next_transition_node->history.push_back(current_arc_length);
                } else {
                    next_transition_node->distance_backward += current_arc_length;
                    next_transition_node->history.push_back(-current_arc_length);
                }

                if(segment_forward) {
                    next_transition_node->distance_forward += distance_to_end;
                    next_transition_node->history.push_back(distance_to_end);
                } else {
                    next_transition_node->distance_backward += distance_to_end;
                    next_transition_node->history.push_back(-distance_to_end);
                }

                Q.insert(next_transition_node);
            }
        }
    }


    std::deque<const Node*> path_transitions;
    Node* gd = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();

    for(Node* candidate : candidates) {
        if(candidate->cost < min_cost) {
            min_cost = candidate->cost;
            gd = candidate;
        }
    }
    while(gd) {
        path_transitions.push_front(gd);
        gd = gd->prev;
    }

    generatePath(start, end,
                 start_segment, end_segment,
                 path_transitions, res);

    return res;
}


void CourseGenerator::generatePath(const path_geom::PathPose& start, const path_geom::PathPose& end,
                                   const Segment* start_segment, const Segment* end_segment,
                                   const std::deque<const Node*>& path_transitions, std::vector<path_geom::PathPose>& res) const
{
    Eigen::Vector2d first_pos = start_segment->line.nearestPointTo(start.pos_);
    const Node* first_node = path_transitions.front();

    Eigen::Vector2d first_delta = first_node->transition->path.front() - first_pos;
    double first_yaw = std::atan2(first_delta(1), first_delta(0));
    res.push_back(path_geom::PathPose(first_pos(0), first_pos(1), first_yaw));

    ROS_INFO_STREAM("generating path from " << path_transitions.size() << " transitions");
    for(const Node* node : path_transitions) {
        const Transition* transition = node->transition;

        if(node->curve_forward) {
            for(std::size_t j = 1, m = transition->path.size(); j < m; ++j) {
                const Eigen::Vector2d& pt = transition->path.at(j);
                const Eigen::Vector2d& prev_pt = transition->path.at(j-1);
                Eigen::Vector2d delta = pt - prev_pt;
                double c_yaw = std::atan2(delta(1), delta(0));
                res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
            }
        } else {
            for(int m = transition->path.size(), j = m-2; j >= 0; --j) {
                const Eigen::Vector2d& pt = transition->path.at(j);
                const Eigen::Vector2d& next_pt = transition->path.at(j+1);
                Eigen::Vector2d delta = pt - next_pt;
                double c_yaw = std::atan2(delta(1), delta(0));
                res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
            }
        }
    }

    Eigen::Vector2d last_pos = end_segment->line.nearestPointTo(end.pos_);
    const Node* last_node = path_transitions.back();

    Eigen::Vector2d last_delta = last_pos - last_node->transition->path.back();
    double last_yaw = std::atan2(last_delta(1), last_delta(0));
    res.push_back(path_geom::PathPose(last_pos(0), last_pos(1), last_yaw));
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

            // check if the segments are already connected
            bool connected = false;
            for(const Transition& t : si.transitions) {
                if(t.target == &sj) {
                    connected = true;
                    break;
                }
            }
            if(connected) {
                continue;
            }

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

                si.transitions.emplace_back(t);
                sj.transitions.emplace_back(t);
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
        for(const Transition& transition : segment.transitions) {
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
