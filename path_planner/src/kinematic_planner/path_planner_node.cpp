/*
 * path_planner_node.hpp
 *
 *  Created on: Apr 3, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef PATH_PLANNER_NODE_HPP
#define PATH_PLANNER_NODE_HPP

/// COMPONENT
#include "../planner_node.h"

/// PROJECT
#include <cslibs_path_planning/generic/Algorithms.hpp>
#include <cslibs_path_planning/generic/ReedsSheppExpansion.hpp>
#include <cslibs_path_planning/common/Bresenham2d.h>
#include <cslibs_path_planning/generic/SteeringNode.hpp>
#include <cslibs_path_planning/generic/SteeringNeighborhood.hpp>
#include <cslibs_path_planning/generic/DynamicSteeringNeighborhood.h>

/// SYSTEM
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>

using namespace lib_path;


// IGNORE END ORIENTATION
template <int distance, int steerangle>
struct NonHolonomicNeighborhoodNoEndOrientation :
        public NonHolonomicNeighborhood<distance, steerangle> {
    typedef NonHolonomicNeighborhood<distance, steerangle> Parent;

    using Parent::distance_step_pixel;

    template <class NodeType>
    static bool isGoal(const NodeType* goal, const NodeType* reference) {
        return std::abs(goal->x - reference->x) <= distance_step_pixel / 2 &&
                std::abs(goal->y - reference->y) <= distance_step_pixel / 2;
    }
};


// MORE PRECISE END POSITION
template <int distance, int steerangle, int moves = NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, bool reversed = false,
          int straight_dir_switch = static_cast<int>(std::round(2.0 / (distance / 100.)))>
struct NonHolonomicNeighborhoodPrecise :
        public NonHolonomicNeighborhood<distance, steerangle, moves, reversed, straight_dir_switch> {
    typedef NonHolonomicNeighborhood<distance, steerangle, moves, reversed, straight_dir_switch> Parent;

    using Parent::distance_step_pixel;

    template <class NodeType>
    static bool isGoal(const NodeType* goal, const NodeType* reference) {
        double delta_rot = std::atan2(std::sin(goal->theta - reference->theta),
                                      std::cos(goal->theta - reference->theta));
        static const double angle_threshold = M_PI / 8;
        if(std::abs(delta_rot) > angle_threshold) {
            return false;
        }

        int delta = 4 * distance;
        if(std::abs(goal->x - reference->x) > delta ||
                std::abs(goal->y - reference->y) > delta) {
            return false;
        }

        double cell_dist = hypot(goal->x - reference->x, goal->y - reference->y);
        double dist = cell_dist * Parent::resolution;

        // euclidean distance
        if(dist < 0.05) {
            return true;
        }


        // check, if goal is near the segment through reference an its predecessor
        if(reference->prev) {
            Eigen::Vector2d p (reference->prev->x, reference->prev->y);
            Eigen::Vector2d r (reference->x, reference->y);
            Eigen::Vector2d g (goal->x, goal->y);

            p *=  Parent::resolution;
            r *=  Parent::resolution;
            g *=  Parent::resolution;

            // calculate distance of goal to the line segment through current and prev
            //  if the projection of goal falls onto the segment, check if the distance is small enough
            double line_distance;
            double l2 = (p-r).squaredNorm();
            if(l2 <= 0.0001) { // v ~= w
                line_distance = (g - p).norm();
            } else {
                double t = (g - p).dot(r - p) / l2;

                t = std::max(0.0, std::min(1.0, t));

                Eigen::Vector2d project = p + t * (r-p);

                line_distance = (g-project).norm();
            }
            return line_distance < 0.1;
        }

        return false;
    }
};




template <typename Algorithm>
struct MapGoalTest
{
    MapGoalTest(Algorithm& algo, const Pose2d& start, int map_search_min_value, int map_search_min_candidates, double min_dist,
                const nav_msgs::OccupancyGrid& map, const lib_path::SimpleGridMap2d* map_info)
        : algo(algo),
          start(start),
          has_heuristic_goal(false),
          min_dist(min_dist),
          map_info(map_info), map(map),
          res(map.info.resolution),
          ox(map.info.origin.position.x),
          oy(map.info.origin.position.y),
          w(map.info.width),
          h(map.info.height),

          candidates(0)
    {
    }

    bool terminate(const typename Algorithm::NodeT* node) const
    {
        double wx, wy;
        map_info->cell2pointSubPixel(node->x,node->y, wx, wy);

        double dist = std::hypot(wx - start.x, wy - start.y);
        if(min_dist > 0.0 && dist < min_dist) {
            return false;
        }

        int x = (-ox + wx) / res;
        int y = (-oy + wy) / res;

        if(x < 0 || x >= w || y < 0 || y >= h) {
            return false;
        }

        int idx = y * w + x;

        const int8_t& val = map.data.at(idx);

        if(val > map_search_min_value) {
            if(has_heuristic_goal) {
                double dist_to_heuristic_goal = std::hypot(wx - heuristic_goal.x, wy - heuristic_goal.y);
                algo.addGoalCandidate(node, dist_to_heuristic_goal);

            } else {
                algo.addGoalCandidate(node, 255 - val);
            }
            ++candidates;
        }

        return candidates > map_search_min_candidates;
    }

    const Pose2d* getHeuristicGoal() const
    {
        if(has_heuristic_goal) {
            return &heuristic_goal;
        } else {
            return NULL;
        }
    }

    void setHeuristicGoal(const Pose2d& goal)
    {
        has_heuristic_goal = true;
        heuristic_goal = goal;
    }

    Algorithm& algo;
    const Pose2d& start;

    bool has_heuristic_goal;
    Pose2d heuristic_goal;

    int map_search_min_value;
    int map_search_min_candidates;
    double min_dist;

    const lib_path::SimpleGridMap2d * map_info;
    const nav_msgs::OccupancyGrid& map;
    double res;
    double ox;
    double oy;
    int w;
    int h;

    mutable int candidates;
};


/**
 * @brief The PathPlanner struct uses our custom planning algorithms to plan paths
 */
struct PathPlanner : public Planner
{
    enum { SCALE = 1 };

    typedef NonHolonomicNeighborhoodPrecise<40, 120> NHNeighbor;
    typedef NonHolonomicNeighborhoodNoEndOrientation<120, 200> NHNeighborNoEndOrientation;


    DEFINE_CONCRETE_ALGORITHM(AStarNoOrientation,
                              Pose2d, GridMap2d, NHNeighbor, NoExpansion,
                              HeuristicL2, DirectionalStateSpaceManager, PriorityQueueManager);

    // PRECOMPILED CONFIGURATIONS
    typedef AStarDynamicHybridHeuristicsSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD_BACKWARD, true>,
    NoExpansion, Pose2d, GridMap2d, 1000> AStarAckermannReversed; // Ackermann
    typedef AStarDynamicSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD_BACKWARD, false>,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSummit; // Summit
    typedef AStarDynamicSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD_BACKWARD, true>,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSummitReversed; // Summit

    typedef AStarDynamicSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD, false>,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSummitForward; // Summit Only Forward
    typedef AStarDynamicSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD, true>,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSummitForwardReversed; // Summit Only Forward

    typedef AStar2dSearch<DirectNeighborhood<8, 1> > AStarOmnidrive; // Omnidrive

    typedef AStarDynamicHybridHeuristicsSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD_BACKWARD, true>, NoExpansion, Pose2d, GridMap2d, 500 > AStarPatsy;
    typedef AStarDynamicHybridHeuristicsSearch<SteeringNeighborhood<40, 2, 15, 60, 120, SteeringMoves::FORWARD, true>, NoExpansion, Pose2d, GridMap2d, 500 > AStarPatsyForward;

    typedef AStarDynamicSearch<DynamicSteeringNeighborhood,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSteeringDynamic; // Generic Steering Planner

    typedef AStarAckermannReversed AStarAckermann;
    typedef AStarOmnidrive AStar2D;

    enum class Algo {
        ACKERMANN = 0,
        SUMMIT = 1,
        SUMMIT_FORWARD = 2,
        OMNI = 3,
        PATSY = 4,
        PATSY_FORWARD = 5,
        GENERIC = 6
    };

    PathPlanner()
        : render_open_cells_(false)
    {
        nh_priv.param("render_open_cells", render_open_cells_, false);

        nh_priv.param("algorithm", algo, std::string("generic"));

        nh_priv.param("oversearch_distance", search_options.oversearch_distance, search_options.oversearch_distance);
        ROS_INFO_STREAM("oversearch distance is " << search_options.oversearch_distance);
        if(std::isnan(search_options.oversearch_distance) || search_options.oversearch_distance > 100) {
            search_options.oversearch_distance = 0.;
        }

        nh_priv.param("penalty/backward", search_options.penalty_backward, search_options.penalty_backward);
        nh_priv.param("penalty/turn", search_options.penalty_turn, search_options.penalty_turn);

        std::transform(algo.begin(), algo.end(), algo.begin(), ::tolower);

        algo_to_use = stringToAlgorithm(algo);

        ROS_INFO_STREAM("planner uses algorithm: " << algo);

        if(render_open_cells_) {
            cell_publisher_ = nh_priv.advertise<nav_msgs::GridCells>("cells", 1, true);
        }
    }

    virtual bool supportsGoalType(int type) const override
    {
        return type == path_msgs::Goal::GOAL_TYPE_POSE ||
                type == path_msgs::Goal::GOAL_TYPE_MAP;
    }

    Algo stringToAlgorithm(const std::string& algo) const
    {
        if(algo == "ackermann") {
            return Algo::ACKERMANN;
        } else if(algo == "summit") {
            return Algo::SUMMIT;
        } else if(algo == "patsy") {
            return Algo::PATSY;
        } else if(algo == "patsy_forward") {
            return Algo::PATSY_FORWARD;
        } else if(algo == "summit_forward") {
            return Algo::SUMMIT_FORWARD;
        } else if(algo == "omni" || algo == "2d") {
            return Algo::OMNI;
        } else if(algo == "generic") {
            return Algo::GENERIC;
        }

        throw std::runtime_error(std::string("unknown algorithm: ") + algo);
    }

    // convert non-directional paths
    path_msgs::PathSequence path2msg(const std::vector<lib_path::HeuristicNode<lib_path::Pose2d>>& path_raw, const ros::Time &goal_timestamp)
    {
        if(path_raw.empty()) {
            return {};
        }

        path_msgs::PathSequence path_out;
        path_out.paths.emplace_back();

        // set timestamp of the received goal for the path message, so they can be associated
        path_out.header.stamp = goal_timestamp;
        path_out.header.frame_id = world_frame_;

        path_msgs::DirectionalPath* path = &path_out.paths.back();

        path->forward = true;

        if(path_raw.size() > 0) {
            for(const auto& next_map : path_raw) {
                geometry_msgs::PoseStamped pose;
                map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                             pose.pose.position.y);

                pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

                path->poses.push_back(pose);
            }
        }

        return path_out;
    }

    // convert directional paths
    template <typename PathT>
    path_msgs::PathSequence path2msg(const PathT& path_raw, const ros::Time &goal_timestamp)
    {
        path_msgs::PathSequence path_out;
        if(path_raw.empty()) {
            return path_out;
        }

        path_out.paths.emplace_back();

        // set timestamp of the received goal for the path message, so they can be associated
        path_out.header.stamp = goal_timestamp;
        path_out.header.frame_id = world_frame_;

        path_msgs::DirectionalPath* path = &path_out.paths.back();

        bool forward = path_raw.front().forward;
        path->forward = forward;

        if(path_raw.size() > 0) {
            for(const auto& next_map : path_raw) {
                geometry_msgs::PoseStamped pose;
                map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                             pose.pose.position.y);

                pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

                if(next_map.forward != forward) {
                    // new segment needed
                    forward = next_map.forward;

                    path_out.paths.emplace_back();
                    path = &path_out.paths.back();

                    path->forward = forward;
                }

                path->poses.push_back(pose);
            }
        }

        return path_out;
    }

    template <typename Algorithm>
    void initSearch (Algorithm& algo,
                     const std_msgs::Header &header,
                     double max_search_duration)
    {
        algo.setMap(map_info);
        algo.setTimeLimit(max_search_duration);

        if(use_cost_map_) {
            cells.header = cost_map.header;
            cells.cell_width = cells.cell_height = cost_map.info.resolution;

            algo.setCostFunction(true);

        } else {
            cells.header = header;
            cells.cell_width = cells.cell_height = 0.05;
        }

        cells.cells.clear();
    }

    template <typename Algorithm>
    path_msgs::PathSequence planInstance (Algo algo_id, Algorithm& algo,
                                          const path_msgs::PlanPathGoal &request,
                                          const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                                          const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        initSearch(algo, request.goal.pose.header, request.options.max_search_duration);

        try {
            typename Algorithm::PathT path;

            algo.setPathCandidateCallback([this, request](const typename Algorithm::PathT& path) {
                path_msgs::PathSequence msg = path2msg(path, request.goal.pose.header.stamp);
                visualizePath(msg, 0, 0.8);
                return false;
            });

            if(render_open_cells_) {
                path = algo.findPath(from_map, to_map,
                                     boost::bind(&PathPlanner::renderCells, this, algo_id),
                                     search_options);

                // render cells once more -> remove the last ones
                renderCells(algo_id);
            } else {
                path = algo.findPath(from_map, to_map, search_options);
            }

            int id = 1;
            for(const typename Algorithm::PathT& path : algo.getPathCandidates()) {
                visualizePath(path2msg(path, request.goal.pose.header.stamp), id++, 0.1);
            }

            return path2msg(path, request.goal.pose.header.stamp);
            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }
        catch(const std::logic_error& e) {
            ROS_ERROR_STREAM("no path found: " << e.what());
        }

        return empty();
    }
    template <typename Algorithm>
    path_msgs::PathSequence planMapInstance (Algo algo_id, Algorithm& algo,
                                             const path_msgs::PlanPathGoal &request,
                                             const Pose2d &from_world, const Pose2d &from_map) {

        initSearch(algo, request.goal.map.header, request.options.max_search_duration);

        try {
            typename Algorithm::PathT path;
            MapGoalTest<Algorithm> goal_test(algo, from_world,
                                             request.goal.map_search_min_value > 0 ? request.goal.map_search_min_value : 32,
                                             request.goal.map_search_min_candidates > 0 ? request.goal.map_search_min_candidates : 64,
                                             request.goal.min_dist,
                                             request.goal.map, map_info);

            if(request.options.has_search_dir) {
                Pose2d goal;
                goal.x = request.options.search_dir.x;
                goal.y = request.options.search_dir.y;
                goal.theta = 0;
                goal_test.setHeuristicGoal(goal);
            }

            if(render_open_cells_) {
                path = algo.findPath(from_map, goal_test,
                                     boost::bind(&PathPlanner::renderCells, this, algo_id),
                                     search_options);

                // render cells once more -> remove the last ones
                renderCells(algo_id);
            } else {
                path = algo.findPath(from_map, goal_test, search_options);
            }

            int id = 1;
            for(const typename Algorithm::PathT& path : algo.getPathCandidates()) {
                visualizePath(path2msg(path, request.goal.pose.header.stamp), id++, 0.1);
            }

            return path2msg(path, ros::Time::now());
            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }
        catch(const std::logic_error& e) {
            ROS_ERROR_STREAM("no path found: " << e.what());
        }
        return empty();
    }

    path_msgs::PathSequence planWithoutTargetPose (const path_msgs::PlanPathGoal &request,
                                                   const Pose2d &from_world, const Pose2d &from_map) {

        Algo algorithm = algo_to_use;

        if(!request.goal.planning_algorithm.data.empty()) {
            ROS_INFO_STREAM("planning w/o target pose with requested algorithm: " << request.goal.planning_algorithm.data);
            algorithm = stringToAlgorithm(request.goal.planning_algorithm.data);
        }

        switch(algorithm) {
        case Algo::ACKERMANN:
            return planMapInstance(algorithm, algo_ackermann, request, from_world, from_map);
        case Algo::SUMMIT:
            return planMapInstance(algorithm, algo_summit_reversed, request, from_world, from_map);
        case Algo::SUMMIT_FORWARD:
            return planMapInstance(algorithm, algo_summit_forward_reversed, request, from_world, from_map);
        case Algo::OMNI:
            return planMapInstance(algorithm, algo_omni, request, from_world, from_map);
        case Algo::GENERIC:
            updateGenericParameters(request);
            return planMapInstance(algorithm, algo_generic, request, from_world, from_map);

        default:
            throw std::runtime_error("unknown algorithm selected");
        }

    }

    path_msgs::PathSequence plan (const path_msgs::PlanPathGoal &goal,
                                  const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                                  const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {
        Algo algorithm = algo_to_use;

        if(!goal.goal.planning_algorithm.data.empty()) {
            ROS_INFO_STREAM("planning w/ target pose with requested algorithm: " << goal.goal.planning_algorithm.data);
            algorithm = stringToAlgorithm(goal.goal.planning_algorithm.data);
        }

        switch(algorithm) {
        case Algo::ACKERMANN:
            return planInstance(algorithm, algo_ackermann, goal, from_world, to_world, from_map, to_map);
        case Algo::SUMMIT:
            return planInstance(algorithm, algo_summit, goal, from_world, to_world, from_map, to_map);
        case Algo::PATSY:
            return planInstance(algorithm, algo_patsy, goal, from_world, to_world, from_map, to_map);
        case Algo::PATSY_FORWARD:
            return planInstance(algorithm, algo_patsy_forward, goal, from_world, to_world, from_map, to_map);
        case Algo::SUMMIT_FORWARD:
            return planInstance(algorithm, algo_summit_forward, goal, from_world, to_world, from_map, to_map);
        case Algo::OMNI:
            return planInstance(algorithm, algo_omni, goal, from_world, to_world, from_map, to_map);
        case Algo::GENERIC:
            updateGenericParameters(goal);
            return planInstance(algorithm, algo_generic, goal, from_world, to_world, from_map, to_map);

        default:
            throw std::runtime_error("unknown algorithm selected");
        }

    }

    void updateGenericParameters(const path_msgs::PlanPathGoal &goal)
    {
        DynamicSteeringNeighborhood::goal_dist_threshold = goal.options.goal_dist_threshold;
        DynamicSteeringNeighborhood::goal_angle_threshold = goal.options.goal_angle_threshold_degree / 180. * M_PI;
        DynamicSteeringNeighborhood::reversed = goal.options.reversed;

        bool forward = goal.options.allow_forward;
        bool backward = goal.options.allow_backward;
        if(!forward && !backward) {
            // if neither direction is allowd (empty message), we default to full planning
            forward = true;
            backward = true;
        }
        DynamicSteeringNeighborhood::allow_forward = forward;
        DynamicSteeringNeighborhood::allow_backward = backward;

        DynamicSteeringNeighborhood::MAX_STEER_ANGLE = goal.options.ackermann_max_steer_angle_degree;
        DynamicSteeringNeighborhood::STEER_DELTA = goal.options.ackermann_steer_delta_degree;
        DynamicSteeringNeighborhood::steer_steps = goal.options.ackermann_steer_steps;
        DynamicSteeringNeighborhood::LA = goal.options.ackermann_la;
    }


    template <class Algorithm>
    void renderCellsInstance(Algorithm& algo)
    {
        if(render_open_cells_) {
            auto& open = algo.getOpenList();

            for(auto it = open.begin(); it != open.end(); ++it) {
                const auto* node = *it;
                geometry_msgs::Point pt;
                map_info->cell2point(node->x, node->y, pt.x, pt.y);
                cells.cells.push_back(pt);
            }

            cell_publisher_.publish(cells);
            cells.cells.clear();
        }
    }

    void renderCells(Algo algo)
    {
        switch(algo) {
        case Algo::ACKERMANN:
            return renderCellsInstance(algo_ackermann);
        case Algo::SUMMIT:
            return renderCellsInstance(algo_summit);
        case Algo::PATSY:
            return renderCellsInstance(algo_patsy);
        case Algo::PATSY_FORWARD:
            return renderCellsInstance(algo_patsy_forward);
        case Algo::SUMMIT_FORWARD:
            return renderCellsInstance(algo_summit_forward);
        case Algo::OMNI:
            return renderCellsInstance(algo_omni);
        case Algo::GENERIC:
            return renderCellsInstance(algo_generic);

        default:
            throw std::runtime_error("unknown algorithm selected");
        }
    }


private:
    SearchOptions search_options;

    AStarAckermann algo_ackermann;
    AStarSummit algo_summit;
    AStarSummit algo_summit_reversed;
    AStarSummitForward algo_summit_forward;
    AStarSummitForward algo_summit_forward_reversed;
    AStarPatsy algo_patsy;
    AStarPatsyForward algo_patsy_forward;
    AStar2D algo_omni;
    AStarSteeringDynamic algo_generic;

    Algo algo_to_use;

    bool render_open_cells_;
    nav_msgs::GridCells cells;
    ros::Publisher cell_publisher_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner", ros::init_options::NoSigintHandler);

    PathPlanner planner;

    ros::WallRate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

#endif // PATH_PLANNER_NODE_HPP
