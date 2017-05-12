#include "path_builder.h"
#include "segment.h"
#include <ros/console.h>

PathBuilder::PathBuilder()
{

}

std::vector<path_geom::PathPose> PathBuilder::build() const
{
    return res;
}

void PathBuilder::addPath(const std::vector<path_geom::PathPose> &path)
{
    res.insert(res.end(), path.begin(), path.end());
}

void PathBuilder::insertTangentPoint(const Segment* segment, Eigen::Vector2d point)
{
    Eigen::Vector2d delta = segment->line.endPoint() - segment->line.startPoint();
    double yaw = std::atan2(delta(1), delta(0));
    res.push_back(path_geom::PathPose(point(0), point(1), yaw));
}

void PathBuilder::extendAlongTargetSegment(const Node* node, double length)
{
    ROS_INFO("extend along next");
    const Transition& current_transition = *node->transition;

    double c_yaw = 0;
    Eigen::Vector2d  pt = current_transition.path.back();

    Eigen::Vector2d ep = current_transition.target->line.endPoint();
    Eigen::Vector2d sp = current_transition.target->line.startPoint();
    Eigen::Vector2d delta = ep - sp;

    c_yaw = std::atan2(delta(1), delta(0));

    Eigen::Vector2d offset(length, 0.0);
    Eigen::Matrix2d rot;
    rot << std::cos(c_yaw), -std::sin(c_yaw), std::sin(c_yaw), std::cos(c_yaw);

    pt += rot * offset;

    res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
}

void PathBuilder::extendAlongSourceSegment(const Node* node, double length)
{
    ROS_INFO("extend along current");
    const Transition& current_transition = *node->transition;

    double c_yaw = 0;
    Eigen::Vector2d pt = current_transition.path.front();


    Eigen::Vector2d ep = current_transition.source->line.endPoint();
    Eigen::Vector2d sp = current_transition.source->line.startPoint();
    Eigen::Vector2d delta = ep - sp;

    c_yaw = std::atan2(delta(1), delta(0)) + M_PI;

    Eigen::Vector2d offset(length, 0.0);
    Eigen::Matrix2d rot;
    rot << std::cos(c_yaw), -std::sin(c_yaw), std::sin(c_yaw), std::cos(c_yaw);

    pt += rot * offset;

    res.push_back(path_geom::PathPose(pt(0), pt(1), c_yaw));
}

void PathBuilder::extendWithStraightTurningSegment(const Eigen::Vector2d& pt, double length)
{
    ROS_INFO("extend straight");
    Eigen::Vector2d prev_pt = res.back().pos_;

    Eigen::Vector2d dir = (pt - prev_pt);
    Eigen::Vector2d offset = dir / dir.norm() * length;
    Eigen::Vector2d pos = pt + offset;

    res.push_back(path_geom::PathPose(pos(0), pos(1), std::atan2(dir(1), dir(0))));
}

void PathBuilder::insertCurveSegment(const Node* node)
{
    const Transition& current_transition = *node->transition;

    if(node->curve_forward) {
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

