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
        const Segment* source;
        const Segment* target;

        Eigen::Vector2d intersection;
        Eigen::Vector2d icr;

        double r;
        double dtheta;

        std::vector<Eigen::Vector2d> path;
    };


    struct Node {
        // associated transition
        const Transition* transition = nullptr;

        bool forward = true;

        // distance travelled until this transition is reached
        double cost = std::numeric_limits<double>::infinity();

        // node via which this transition is reached
        Node* prev = nullptr;


        double distance_forward = 0.0;
        double distance_backward = 0.0;

        std::vector<double> history;
    };


public:
    CourseGenerator(ros::NodeHandle& nh);

    void createMap(const XmlRpc::XmlRpcValue& map_segment_array);
    void publishMarkers() const;

    bool hasSegments() const;

    std::vector<path_geom::PathPose> findPath(const path_geom::PathPose& start, const path_geom::PathPose& end) const;

private:
    static Vector2d readPoint(const XmlRpc::XmlRpcValue& value, int index);

    void addLines(visualization_msgs::MarkerArray &array) const;
    void addTransitions(visualization_msgs::MarkerArray &array) const;
    void addIntersections(visualization_msgs::MarkerArray& array) const;

    const Segment* findClosestSegment(const path_geom::PathPose& pose, double yaw_tolerance, double max_dist) const;

    void generatePath(const path_geom::PathPose &start, const path_geom::PathPose &end,
                      const Segment *start_segment, const Segment *end_segment,
                      const std::deque<const Node *> &path_transitions, std::vector<path_geom::PathPose>& res) const;


private:
    std::vector<Segment> segments_;
    std::vector<Eigen::Vector2d> intersections_;

    ros::NodeHandle& nh_;
    ros::Publisher pub_viz_;
};

#endif // COURSE_GENERATOR_H
