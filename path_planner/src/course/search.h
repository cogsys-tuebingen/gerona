#ifndef SEARCH_H
#define SEARCH_H

#include <utils_path/geometry/line.h>
#include <XmlRpcValue.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/common/SimpleGridMap2d.h>
#include <nav_msgs/OccupancyGrid.h>

#include "node.h"

class CourseGenerator;

class Search
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

public:
    Search(CourseGenerator& generator);

    std::vector<path_geom::PathPose> findPath(const path_geom::PathPose& start, const path_geom::PathPose& end);

private:

private:
    void initMaps(const nav_msgs::OccupancyGrid& map);

    void generatePathCandidate(Node* current_node);
    void generatePath(const std::deque<const Node *> &path_transitions, std::vector<path_geom::PathPose>& res) const;

    double calculateStraightCost(Node* current_node, const Vector2d &start_point_on_segment, const Vector2d &end_point_on_segment) const;
    double calculateCurveCost(Node* current_node) const;

    template <typename NodeT>
    path_geom::PathPose convertToWorld(const NodeT& node);

    lib_path::Pose2d convertToMap(const path_geom::PathPose& node);

    template <typename PathT>
    std::vector<path_geom::PathPose> combine(const PathT& start,
                 const std::vector<path_geom::PathPose>& centre,
                 const PathT& end);


    void insertCurveSegment(std::vector<path_geom::PathPose>& res, const Node* current_node) const;
    void extendAlongSourceSegment(std::vector<path_geom::PathPose>& res, const Node *current_node) const;
    void extendAlongTargetSegment(std::vector<path_geom::PathPose>& res, const Node *current_node) const;
    void extendWithStraightTurningSegment(std::vector<path_geom::PathPose>& res, const Vector2d &pt) const;


    Eigen::Vector2d findStartPointOnNextSegment(const Node* node) const;
    Eigen::Vector2d findStartPointOnNextSegment(const Node* node, const Transition* transition) const;
    Eigen::Vector2d findEndPointOnNextSegment(const Node* node) const;
    Eigen::Vector2d findEndPointOnSegment(const Node* node, const Transition* transition) const;

    bool isNextSegmentForward(const Node* node) const;
    double effectiveLengthOfNextSegment(const Node* node) const;
    bool isSegmentForward(const Segment* segment, const Eigen::Vector2d& pos, const Eigen::Vector2d& target) const;
    bool isPreviousSegmentForward(Node* current_node) const;

    std::string signature(const Node* head) const;

private:
    ros::NodeHandle pnh_;

    CourseGenerator& generator_;

    ros::ServiceClient map_service_client_;

    AStarPatsyForward algo_forward;
    AStarPatsyForwardTurning algo_forward_turn;
    AStarPatsyReversed algo_reverse;
    AStarPatsyReversedTurning algo_reverse_turn;
    std::shared_ptr<lib_path::SimpleGridMap2d> map_info;


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

    Eigen::Vector2d start_pt;
    Eigen::Vector2d end_pt;

    double min_cost;
    std::vector<path_geom::PathPose> best_path;
};

#endif // SEARCH_H
