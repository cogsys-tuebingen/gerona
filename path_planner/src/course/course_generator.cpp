#include "course_generator.h"
#include <utils_path/geometry/intersector.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>


CourseGenerator::CourseGenerator(ros::NodeHandle &nh)
    : nh_(nh), pnh_("~")
{
    pub_viz_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, true);

    pnh_.param("course/radius", curve_radius, 1.0);
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

Segment CourseGenerator::readSegment(const XmlRpc::XmlRpcValue &map_segment_array, int index)
{
    XmlRpc::XmlRpcValue segment = map_segment_array[index];

    if(segment.size() != 2) {
        ROS_FATAL_STREAM("segment " << segment << " has an invalid format, should be [start, end]");
        std::abort();
    }

    Eigen::Vector2d start = readPoint(segment, 0);
    Eigen::Vector2d end = readPoint(segment, 1);

    path_geom::Line line(start, end);

    return Segment(line);
}


const Segment* CourseGenerator::findClosestSegment(const path_geom::PathPose &pose, double yaw_tolerance, double max_dist) const
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
        segments_.emplace_back(readSegment(map_segment_array, i));
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
                // only forward motion is allowed
                //  -> if the intersection is at the start of the source segment -> ignore
                if((intersection - si.line.startPoint()).norm() < eps) {
                    continue;
                }
                //  -> if the intersection is at the end of the target segment -> ignore
                if((intersection - sj.line.endPoint()).norm() < eps) {
                    continue;
                }

                addTransition(si, sj, intersection);
            }
        }
    }
}


void CourseGenerator::addTransition(Segment &from, Segment &to, const Eigen::Vector2d& intersection)
{
    intersections_.push_back(intersection);

    Eigen::Vector2d icr = calculateICR(from, to, intersection);

    Transition t;
    t.source = &from;
    t.target = &to;
    t.icr = icr;
    t.intersection = intersection;
    t.r = curve_radius;
    t.dtheta = calculateSpan(from, to, icr);
    t.path = calculateCurvePoints(from, to, icr, t.dtheta);

    from.forward_transitions.emplace_back(t);
    to.backward_transitions.emplace_back(t);
}

Vector2d CourseGenerator::calculateICR(const Segment &from, const Segment &to, const Eigen::Vector2d& intersection) const
{
    Eigen::Vector2d a = from.line.endPoint() - from.line.startPoint();
    Eigen::Vector2d b = to.line.endPoint() - to.line.startPoint();

    Eigen::Vector2d an = a / a.norm();
    Eigen::Vector2d bn = b / b.norm();

    double opening_angle = MathHelper::NormalizeAngle(std::atan2(bn(1), bn(0)) - std::atan2(an(1), an(0)));
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

    return icr;
}

double CourseGenerator::calculateSpan(const Segment &from, const Segment &to, const Eigen::Vector2d& icr) const
{
    Eigen::Vector2d s = from.line.projectPoint(icr);
    Eigen::Vector2d e = to.line.projectPoint(icr);

    Eigen::Vector2d ds = s - icr;
    Eigen::Vector2d de = e - icr;

    double start_angle = std::atan2(ds(1), ds(0));
    double end_angle = std::atan2(de(1), de(0));

    return MathHelper::NormalizeAngle(end_angle - start_angle);
}

std::vector<Eigen::Vector2d> CourseGenerator::calculateCurvePoints(const Segment &from, const Segment &to, const Eigen::Vector2d& icr, double dtheta) const
{
    std::vector<Eigen::Vector2d> result;

    Eigen::Vector2d first_pt = from.line.projectPoint(icr);
    Eigen::Vector2d last_pt = to.line.projectPoint(icr);

    double desired_step = M_PI / 32.0;

    int segments = std::abs((int) std::round(dtheta / desired_step));

    if(segments <= 1) {
        result.push_back(first_pt);
        result.push_back(last_pt);

    } else {
        double step_size = dtheta / segments;

        Eigen::Matrix2d delta;
        delta << std::cos(step_size), -std::sin(step_size), std::sin(step_size), std::cos(step_size);

        Eigen::Vector2d offset = first_pt - icr;

        for(int step = 0; step < segments; ++step) {
            result.push_back(icr + offset);

            offset = delta * offset;
        }
    }

    return result;
}

void CourseGenerator::publishMarkers() const
{
    visualization_msgs::MarkerArray array;

    addLineMarkers(array);
    addIntersectionMarkers(array);
    addTransitionMarkers(array);


    pub_viz_.publish(array);
}

bool CourseGenerator::hasSegments() const
{
    return !segments_.empty();
}

void CourseGenerator::addLineMarkers(visualization_msgs::MarkerArray& array) const
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

void CourseGenerator::addTransitionMarkers(visualization_msgs::MarkerArray& array) const
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

void CourseGenerator::addIntersectionMarkers(visualization_msgs::MarkerArray& array) const
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

const std::vector<Segment>& CourseGenerator::getSegments() const
{
    return segments_;
}
