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
#include "analyzer.h"

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

    friend class Analyzer;

public:
    Search(const CourseGenerator& generator);

    std::vector<path_geom::PathPose> findPath(const path_geom::PathPose& start, const path_geom::PathPose& end);

private:
    void initMaps(const nav_msgs::OccupancyGrid& map);

    template <typename AlgorithmForward, typename AlgorithmFull>
    std::vector<path_geom::PathPose> findAppendix(const nav_msgs::OccupancyGrid &map, const path_geom::PathPose& pose, const std::string& type);
    bool findAppendices(const nav_msgs::OccupancyGrid &map, const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose);

    std::vector<path_geom::PathPose> performDijkstraSearch();
    void initNodes();

    void enqueueStartingNodes(std::set<Node *, bool(*)(const Node *, const Node *)> &queue);

    void generatePathCandidate(Node* current_node);
    void generatePath(const std::deque<const Node *> &path_transitions, PathBuilder &path_builder) const;

    path_geom::PathPose convertToWorld(const NodeT& node);
    lib_path::Pose2d convertToMap(const path_geom::PathPose& node);

private:
    ros::NodeHandle pnh_;

    Analyzer analyzer;
    const CourseGenerator& generator_;

    // appendix parameters
    ros::ServiceClient map_service_client_;
    std::shared_ptr<lib_path::SimpleGridMap2d> map_info;

    double size_forward;
    double size_backward;
    double size_width;

    std::vector<path_geom::PathPose> start_appendix;
    std::vector<path_geom::PathPose> end_appendix;

    // search parameters
    std::map<const Transition*, Node> nodes;

    const Segment* start_segment;
    const Segment* end_segment;
    Eigen::Vector2d start_pt;
    Eigen::Vector2d end_pt;

    double min_cost;
    std::vector<path_geom::PathPose> best_path;
};

#endif // SEARCH_H
