#ifndef COURSE_GENERATOR_H
#define COURSE_GENERATOR_H

#include <utils_path/geometry/line.h>
#include <XmlRpcValue.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/common/SimpleGridMap2d.h>

class CourseGenerator
{
public:
//    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 120, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, false>,
//    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForward;
//    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 120, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
//    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversed;
    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, false>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForward;
    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, true>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversed;

    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, false>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForwardTurning;
    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversedTurning;

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

        const Segment* associated_segment = nullptr;

        bool curve_forward= true;

        // distance travelled until this transition is reached
        double cost = std::numeric_limits<double>::infinity();

        // node via which this transition is reached
        Node* prev = nullptr;
        Node* next = nullptr;


        double distance_forward = 0.0;
        double distance_backward = 0.0;

        std::vector<double> history;
    };


public:
    CourseGenerator(ros::NodeHandle& nh);
    ~CourseGenerator();

    void createMap(const XmlRpc::XmlRpcValue& map_segment_array);
    void publishMarkers() const;

    bool hasSegments() const;

    const Segment* findClosestSegment(const path_geom::PathPose& pose, double yaw_tolerance, double max_dist) const;

    std::vector<path_geom::PathPose> findPath(const path_geom::PathPose& start, const path_geom::PathPose& end);


private:
    static Vector2d readPoint(const XmlRpc::XmlRpcValue& value, int index);

    void addLines(visualization_msgs::MarkerArray &array) const;
    void addTransitions(visualization_msgs::MarkerArray &array) const;
    void addIntersections(visualization_msgs::MarkerArray& array) const;

    void generatePath(const std::deque<const Node *> &path_transitions, std::vector<path_geom::PathPose>& res) const;



    template <typename NodeT>
    path_geom::PathPose convertToWorld(const NodeT& node);

    lib_path::Pose2d convertToMap(const path_geom::PathPose& node);

    template <typename PathT>
    std::vector<path_geom::PathPose> combine(const PathT& start,
                 const std::vector<path_geom::PathPose>& centre,
                 const PathT& end);


    void insertCurveSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const;
    void insertStraightTurningSegment(std::vector<path_geom::PathPose>& res, const Node *current_node) const;


    bool isAssociatedSegmentForward(const Node* node) const;
    bool isSegmentForward(const CourseGenerator::Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target) const;

private:

    std::vector<Segment> segments_;
    std::vector<Eigen::Vector2d> intersections_;

    ros::NodeHandle& nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_viz_;

    ros::ServiceClient map_service_client_;

    AStarPatsyForward algo_forward;
    AStarPatsyForwardTurning algo_forward_turn;
    AStarPatsyReversed algo_reverse;
    AStarPatsyReversedTurning algo_reverse_turn;
    std::shared_ptr<lib_path::SimpleGridMap2d> map_info;

    double curve_radius;

    double backward_penalty_factor;
    double turning_straight_segment;
    double turning_penalty;

    double size_forward;
    double size_backward;
    double size_width;


    const Segment* start_segment;
    const Segment* end_segment;

    path_geom::PathPose start;
    path_geom::PathPose end;
};

#endif // COURSE_GENERATOR_H
