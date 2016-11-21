#ifndef COURSE_MAP_H
#define COURSE_MAP_H

#include <cslibs_path_planning/geometry/line.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

#include "segment.h"

class CourseMap
{
public:
    CourseMap(ros::NodeHandle& nh);
    ~CourseMap();

    void load(const XmlRpc::XmlRpcValue& map_segment_array);
    void publishMarkers() const;

    const Segment* findClosestSegment(const path_geom::PathPose& pose, double yaw_tolerance, double max_dist) const;

    bool hasSegments() const;
    const std::vector<Segment> &getSegments() const;

private:
    static Eigen::Vector2d readPoint(const XmlRpc::XmlRpcValue& value, int index);
    static Segment readSegment(const XmlRpc::XmlRpcValue& value, int index);

    void addTransition(Segment &from, Segment &to, const Eigen::Vector2d &intersection);

    Eigen::Vector2d calculateICR(const Segment &from, const Segment &to, const Eigen::Vector2d& intersection) const;
    double calculateSpan(const Segment &from, const Segment &to, const Eigen::Vector2d &icr) const;
    std::vector<Eigen::Vector2d> calculateCurvePoints(const Segment &from, const Segment &to, const Eigen::Vector2d &icr, double dtheta) const;

    void addLineMarkers(visualization_msgs::MarkerArray &array) const;
    void addTransitionMarkers(visualization_msgs::MarkerArray &array) const;
    void addIntersectionMarkers(visualization_msgs::MarkerArray& array) const;

private:
    std::vector<Segment> segments_;
    std::vector<Eigen::Vector2d> intersections_;

    ros::NodeHandle& nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_viz_;

    double curve_radius;
};

#endif // COURSE_MAP_H
