#ifndef COURSE_GENERATOR_H
#define COURSE_GENERATOR_H

#include <utils_path/geometry/line.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

#include "segment.h"

class CourseGenerator
{
public:
    CourseGenerator(ros::NodeHandle& nh);
    ~CourseGenerator();

    void createMap(const XmlRpc::XmlRpcValue& map_segment_array);
    void publishMarkers() const;

    const Segment* findClosestSegment(const path_geom::PathPose& pose, double yaw_tolerance, double max_dist) const;

    bool hasSegments() const;
    const std::vector<Segment> &getSegments() const;

private:
    static Vector2d readPoint(const XmlRpc::XmlRpcValue& value, int index);
    static Segment readSegment(const XmlRpc::XmlRpcValue& value, int index);

    void addTransition(Segment &from, Segment &to, const Vector2d &intersection);

    Eigen::Vector2d calculateICR(const Segment &from, const Segment &to, const Eigen::Vector2d& intersection) const;
    double calculateSpan(const Segment &from, const Segment &to, const Vector2d &icr) const;
    std::vector<Eigen::Vector2d> calculateCurvePoints(const Segment &from, const Segment &to, const Vector2d &icr, double dtheta) const;

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

#endif // COURSE_GENERATOR_H
