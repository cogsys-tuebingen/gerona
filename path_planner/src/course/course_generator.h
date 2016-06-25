#ifndef COURSE_GENERATOR_H
#define COURSE_GENERATOR_H

#include <utils_path/geometry/line.h>
#include <XmlRpcValue.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

class CourseGenerator
{
public:
    CourseGenerator(ros::NodeHandle& nh);

    void createMap(const XmlRpc::XmlRpcValue& map_segment_array);
    void publishMarkers() const;

private:
    static Vector2d readPoint(const XmlRpc::XmlRpcValue& value, int index);

    void addLines(visualization_msgs::MarkerArray &array) const;
    void addTransitions(visualization_msgs::MarkerArray &array) const;
    void addIntersections(visualization_msgs::MarkerArray& array) const;

private:
    class Transition;

    class Segment {
    public:
        path_geom::Line line;
        std::vector<Transition> transitions;

        Segment(const path_geom::Line& line)
            : line(line)
        {
        }
    };

    class Transition {
    public:
        const Segment* target;
        Eigen::Vector2d intersection;
        Eigen::Vector2d icr;
        double r;
        std::vector<Eigen::Vector2d> path;
    };

private:
    std::vector<Segment> segments_;
    std::vector<Eigen::Vector2d> intersections_;

    ros::NodeHandle& nh_;
    ros::Publisher pub_viz_;

};

#endif // COURSE_GENERATOR_H
