#ifndef SEARCH_H
#define SEARCH_H

#include <utils_path/geometry/shape.h>
#include <XmlRpcValue.h>
#include <ros/node_handle.h>
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/common/SimpleGridMap2d.h>
#include <nav_msgs/OccupancyGrid.h>

#include "node.h"
#include "path_builder.h"

class CourseGenerator;

class Search
{
public:
    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, false>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForward;
    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, true>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversed;

    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, false>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForwardTurning;
    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversedTurning;

    typedef AStarPatsyForward::NodeT NodeT;

public:
    Search(const CourseGenerator& generator);

    std::vector<path_geom::PathPose> findPath(const path_geom::PathPose& start, const path_geom::PathPose& end);

private:
    void initMaps(const nav_msgs::OccupancyGrid& map);

    bool findAppendices(const nav_msgs::OccupancyGrid &map, const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose);

    template <typename AlgorithmForward, typename AlgorithmFull>
    std::vector<path_geom::PathPose> findAppendix(const nav_msgs::OccupancyGrid &map, const path_geom::PathPose& pose, const std::string& type);


    std::vector<path_geom::PathPose> performDijkstraSearch();
    void initNodes();

    void enqueueStartingNodes(std::set<Node *, bool(*)(const Node *, const Node *)> &queue);

    void generatePathCandidate(Node* current_node);
    void generatePath(const std::deque<const Node *> &path_transitions, PathBuilder &path_builder) const;

    double calculateStraightCost(Node* current_node, const Vector2d &start_point_on_segment, const Vector2d &end_point_on_segment) const;
    double calculateCurveCost(Node* current_node) const;

    path_geom::PathPose convertToWorld(const NodeT& node);
    lib_path::Pose2d convertToMap(const path_geom::PathPose& node);

    Eigen::Vector2d findStartPointOnNextSegment(const Node* node) const;
    Eigen::Vector2d findStartPointOnNextSegment(const Node* node, const Transition* transition) const;
    Eigen::Vector2d findEndPointOnNextSegment(const Node* node) const;
    Eigen::Vector2d findEndPointOnSegment(const Node* node, const Transition* transition) const;

    bool isNextSegmentForward(const Node* node) const;
    double effectiveLengthOfNextSegment(const Node* node) const;
    bool isSegmentForward(const Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target) const;
    bool isPreviousSegmentForward(const Node* current_node) const;
    bool isStartSegmentForward(const Node *node) const;

    std::string signature(const Node* head) const;

private:
    ros::NodeHandle pnh_;

    const CourseGenerator& generator_;

    ros::ServiceClient map_service_client_;

    std::shared_ptr<lib_path::SimpleGridMap2d> map_info;


    double backward_penalty_factor;
    double turning_straight_segment;
    double turning_penalty;

    double size_forward;
    double size_backward;
    double size_width;

    std::map<const Transition*, Node> nodes;

    const Segment* start_segment;
    const Segment* end_segment;
    Eigen::Vector2d start_pt;
    Eigen::Vector2d end_pt;


    double min_cost;
    std::vector<path_geom::PathPose> best_path;

    std::vector<path_geom::PathPose> start_appendix;
    std::vector<path_geom::PathPose> end_appendix;
};

#endif // SEARCH_H
