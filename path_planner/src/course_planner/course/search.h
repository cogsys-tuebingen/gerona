#ifndef SEARCH_H
#define SEARCH_H

#include <cslibs_path_planning/geometry/shape.h>
#include <XmlRpcValue.h>
#include <ros/node_handle.h>
#include <cslibs_path_planning/generic/Algorithms.hpp>
#include <cslibs_path_planning/common/SimpleGridMap2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cslibs_path_planning/generic/SteeringNode.hpp>
#include <cslibs_path_planning/generic/SteeringNeighborhood.hpp>
#include <path_msgs/PathSequence.h>
#include <path_msgs/PlanPathGoal.h>

#include "node.h"
#include "path_builder.h"
#include "cost_calculator.h"

class CourseMap;

class Search
{
public:
//    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, false>,
//    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForward;
//    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, true>,
//    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversed;

//    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, false>,
//    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForwardTurning;
//    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
//    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversedTurning;

    typedef lib_path::AStarDynamicSearch<lib_path::SteeringNeighborhood<40, 4, 15, 60, 120, lib_path::SteeringMoves::FORWARD, false>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForward;
    typedef lib_path::AStarDynamicSearch<lib_path::SteeringNeighborhood<40, 4, 15, 60, 120, lib_path::SteeringMoves::FORWARD, true>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversed;

    typedef lib_path::AStarDynamicSearch<lib_path::SteeringNeighborhood<40, 4, 15, 60, 120, lib_path::SteeringMoves::FORWARD_BACKWARD, false>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForwardTurning;
    typedef lib_path::AStarDynamicSearch<lib_path::SteeringNeighborhood<40, 4, 15, 60, 120, lib_path::SteeringMoves::FORWARD_BACKWARD, true>,
    lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyReversedTurning;

    typedef AStarPatsyForward::NodeT NodeT;

    friend class Analyzer;

public:
    Search(const CourseMap& generator);

    path_msgs::PathSequence findPath(lib_path::SimpleGridMap2d *map, const path_msgs::PlanPathGoal &goal, const path_geom::PathPose& start, const path_geom::PathPose& end);

private:
    path_msgs::PathSequence tryDirectPath(const path_geom::PathPose& start, const path_geom::PathPose& end);

    template <typename AlgorithmForward, typename AlgorithmFull>
    path_msgs::PathSequence findAppendix(const path_geom::PathPose& pose, const std::string& type);
    bool findAppendices(const path_geom::PathPose& start_pose, const path_geom::PathPose& end_pose);

    path_msgs::PathSequence performDijkstraSearch();
    void initNodes();

    void enqueueStartingNodes(std::set<Node *, bool(*)(const Node *, const Node *)> &queue);

    void generatePathCandidate(Node* current_node);
    void generatePath(const std::deque<const Node *> &path_transitions, PathBuilder &path_builder) const;

    path_geom::PathPose convertToWorld(const NodeT& node);
    path_msgs::PathSequence convertToWorld(const std::vector<NodeT>& path);
    lib_path::Pose2d convertToMap(const path_geom::PathPose& node);

private:
    ros::NodeHandle pnh_;

    CostCalculator cost_calculator_;
    const CourseMap& generator_;

    // appendix parameters
    lib_path::SimpleGridMap2d* map_info;

    std::string world_frame_;
    ros::Time time_stamp_;

    double size_forward;
    double size_backward;
    double size_width;

    double max_distance_for_direct_try;
    double max_time_for_direct_try;

    path_msgs::PathSequence start_appendix;
    path_msgs::PathSequence end_appendix;

    // search parameters
    std::map<const Transition*, Node> nodes;

    const Segment* start_segment;
    const Segment* end_segment;
    Eigen::Vector2d start_pt;
    Eigen::Vector2d end_pt;

    double min_cost;
    path_msgs::PathSequence best_path;
};

#endif // SEARCH_H
