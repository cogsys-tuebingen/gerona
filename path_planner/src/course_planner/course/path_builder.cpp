#include "path_builder.h"
#include "segment.h"
#include <ros/console.h>
#include <tf/tf.h>

PathBuilder::PathBuilder(Search& search)
    : Analyzer(search)
{

}

path_msgs::PathSequence PathBuilder::build() const
{
    return res;
}

void PathBuilder::addPath(const path_msgs::PathSequence &path)
{
    res.paths.insert(res.paths.end(), path.paths.begin(), path.paths.end());
}

void PathBuilder::insertTangentPoint(const Segment* segment, Eigen::Vector2d point)
{
    ROS_DEBUG("insert tangent point");
    Eigen::Vector2d delta = segment->line.endPoint() - segment->line.startPoint();
    double yaw = std::atan2(delta(1), delta(0));
    if(res.paths.empty()) {
        res.paths.emplace_back();
        // TODO: somehow calculate the initial motion direction
        res.paths.back().forward = true;

    } else {
        const geometry_msgs::PoseStamped& last = res.paths.back().poses.back();
        Eigen::Vector2d start(last.pose.position.x, last.pose.position.y);
        bool forward = isSegmentForward(segment, start, point);
        if(forward != res.paths.back().forward) {
            res.paths.emplace_back();
            res.paths.back().forward = forward;
        }
    }

    insertPose(point, yaw);
}

void PathBuilder::extendAlongTargetSegment(const Node* node, double length)
{
    ROS_DEBUG("extend along next");
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

    bool forward = isPreviousSegmentForward(node);
    if(res.paths.empty()|| res.paths.back().forward != forward) {
        res.paths.emplace_back();
        res.paths.back().forward = forward;
    }

    insertPose(pt, c_yaw);
}

void PathBuilder::extendAlongSourceSegment(const Node* node, double length)
{
    ROS_DEBUG("extend along current");
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


    bool forward = isPreviousSegmentForward(node);
    if(res.paths.empty() || res.paths.back().forward != forward) {
        res.paths.emplace_back();
        res.paths.back().forward = forward;
    }

    insertPose(pt, c_yaw);
}

void PathBuilder::extendWithStraightTurningSegment(const Eigen::Vector2d& pt, double length)
{
    ROS_DEBUG("extend straight");
    geometry_msgs::PoseStamped prev = res.paths.back().poses.back();
    Eigen::Vector2d prev_pt(prev.pose.position.x, prev.pose.position.y);

    Eigen::Vector2d dir = (pt - prev_pt);
    Eigen::Vector2d offset = dir / dir.norm() * length;
    Eigen::Vector2d pos = pt + offset;

    if(res.paths.empty()) {
        throw std::runtime_error("cannot extend straight with an empty path");
    }

    insertPose(pos, std::atan2(dir(1), dir(0)));
}

void PathBuilder::insertCurveSegment(const Node* node)
{
    ROS_DEBUG("insert curve segment");
    const Transition& current_transition = *node->transition;

    bool curve_forward = node->curve_forward;
    bool prev_forward = isPreviousSegmentForward(node);

    bool was_forward = res.paths.back().forward;

    if(curve_forward != was_forward) {
        // change direction before the curve
        res.paths.emplace_back();
        res.paths.back().forward = curve_forward;

    } else if(prev_forward == curve_forward) {
        // one straight motion, continue on the last path
        ROS_DEBUG_STREAM("keep direction " << (bool) curve_forward << " in curve, last is " << (bool) was_forward);
    } else {
        // change direction in the curve
        ROS_DEBUG_STREAM("turn in curve, last is " << (bool) res.paths.back().forward);
        res.paths.emplace_back();
        res.paths.back().forward = curve_forward;
    }


    if(res.paths.empty()) {
        throw std::runtime_error("cannot extend with a curve with an empty path");
    }


    if(node->curve_forward) {
        ROS_DEBUG("insert curve: curve is forward");
        for(std::size_t j = 1, m = current_transition.path.size(); j < m; ++j) {
            const Eigen::Vector2d& pt = current_transition.path.at(j);
            const Eigen::Vector2d& prev_pt = current_transition.path.at(j-1);
            Eigen::Vector2d delta = pt - prev_pt;
            double c_yaw = std::atan2(delta(1), delta(0));

            insertPose(pt, c_yaw);
        }

    } else {
        ROS_DEBUG("insert curve: curve is backward");
        for(int m = current_transition.path.size(), j = m-2; j >= 0; --j) {
            const Eigen::Vector2d& pt = current_transition.path.at(j);
            const Eigen::Vector2d& next_pt = current_transition.path.at(j+1);
            Eigen::Vector2d delta = pt - next_pt;
            double c_yaw = std::atan2(delta(1), delta(0));

            insertPose(pt, c_yaw);
        }
    }
}

void PathBuilder::insertPose(const Eigen::Vector2d &pt, double c_yaw)
{
    geometry_msgs::PoseStamped point;
    point.pose.position.x = pt(0);
    point.pose.position.y = pt(1);
    point.pose.orientation = tf::createQuaternionMsgFromYaw(c_yaw);
    res.paths.back().poses.push_back(point);
}
