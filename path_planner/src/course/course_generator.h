#ifndef COURSE_GENERATOR_H
#define COURSE_GENERATOR_H

#include <utils_path/geometry/line.h>
#include <XmlRpcValue.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/common/SimpleGridMap2d.h>
#include <nav_msgs/OccupancyGrid.h>

#include "node.h"
#include "transition.h"
#include "segment.h"

class CourseGenerator
{
public:
    CourseGenerator(ros::NodeHandle& nh);
    ~CourseGenerator();

    void createMap(const XmlRpc::XmlRpcValue& map_segment_array);
    void publishMarkers() const;

    bool hasSegments() const;

    const Segment* findClosestSegment(const path_geom::PathPose& pose, double yaw_tolerance, double max_dist) const;

    const std::vector<Segment> &getSegments() const;

private:
    static Vector2d readPoint(const XmlRpc::XmlRpcValue& value, int index);

    void addLines(visualization_msgs::MarkerArray &array) const;
    void addTransitions(visualization_msgs::MarkerArray &array) const;
    void addIntersections(visualization_msgs::MarkerArray& array) const;

private:

    std::vector<Segment> segments_;
    std::vector<Eigen::Vector2d> intersections_;

    ros::NodeHandle& nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_viz_;

    double curve_radius;
};

#endif // COURSE_GENERATOR_H
