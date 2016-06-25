#include "course_generator.h"
#include <utils_path/geometry/intersector.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

CourseGenerator::CourseGenerator(ros::NodeHandle &nh)
    : nh_(nh)
{
    pub_viz_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, true);
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

    double curve_radius = 0.75;

    for(std::size_t i = 0; i < segments_.size(); ++i) {
        for(std::size_t j = 1; j < segments_.size(); ++j) {
            if(i == j) {
                continue;
            }

            // check for intersection si to sj
            Segment &si = segments_[i];
            const Segment &sj = segments_[j];

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

                double span = MathHelper::NormalizeAngle(end_angle - start_angle);

                double desired_step = M_PI / 32.0;

                int segments = std::abs((int) std::round(span / desired_step));


                if(segments <= 1) {
                    t.path.push_back(s);
                    t.path.push_back(e);

                } else {
                    double step_size = span / segments;

                    Eigen::Matrix2d delta;
                    delta << std::cos(step_size), -std::sin(step_size), std::sin(step_size), std::cos(step_size);

                    Eigen::Vector2d offset = ds;

                    for(int step = 0; step < segments; ++step) {
                        t.path.push_back(icr + offset);

                        offset = delta * offset;
                    }
                }

                si.transitions.emplace_back(t);
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
