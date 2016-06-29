/*
 * path_planner_node.hpp
 *
 *  Created on: Apr 3, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef PATH_PLANNER_NODE_HPP
#define PATH_PLANNER_NODE_HPP

/// COMPONENT
#include "planner_node.h"

/// PROJECT
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/generic/ReedsSheppExpansion.hpp>
#include <utils_path/common/Bresenham2d.h>

/// SYSTEM
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>

using namespace lib_path;


// IGNORE END ORIENTATION
template <int n, int distance>
struct NonHolonomicNeighborhoodNoEndOrientation :
        public NonHolonomicNeighborhood<n, distance> {
    typedef NonHolonomicNeighborhood<n, distance> Parent;

    using Parent::distance_step_pixel;

    template <class NodeType>
    static bool isGoal(const NodeType* goal, const NodeType* reference) {
        return std::abs(goal->x - reference->x) <= distance_step_pixel / 2 &&
                std::abs(goal->y - reference->y) <= distance_step_pixel / 2;
    }
};


// MORE PRECISE END POSITION
template <int n, int distance, int moves = NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, bool reversed = false>
struct NonHolonomicNeighborhoodPrecise :
        public NonHolonomicNeighborhood<n, distance, moves, reversed> {
    typedef NonHolonomicNeighborhood<n, distance, moves, reversed> Parent;

    using Parent::distance_step_pixel;

    template <class NodeType>
    static bool isGoal(const NodeType* goal, const NodeType* reference) {
        double delta_rot = std::atan2(std::sin(goal->theta - reference->theta),
                                      std::cos(goal->theta - reference->theta));
        static const double angle_threshold = M_PI / 8;
        if(std::abs(delta_rot) > angle_threshold) {
            return false;
        }

        double cell_dist = hypot(goal->x - reference->x, goal->y - reference->y);
        double dist = cell_dist * Parent::resolution;

        // euclidean distance
        int delta = 2;
        if(dist < 0.7) return true;

        if(std::abs(goal->x - reference->x) < delta &&
                std::abs(goal->y - reference->y) < delta) {
            return true;
        }

        return false;
        // check, if goal is between reference an its predecessor
        if(reference->prev) {
            Pose2d v (*reference->prev);
            Pose2d w (*reference);
            Pose2d p (*goal);

            double line_distance;
            double l2 = (v-w).distance_to_origin();
            if(l2 <= 0.0001) { // v ~= w
                line_distance = p.distance_to(v);
            } else {
                double t = dot(p - v, w - v) / l2;

                if(t < 0.0) {
                    return false;
                } else if(t > 1.0) {
                    return false;
                } else {
                    Pose2d project = v + t * (w-v);

                    line_distance = p.distance_to(project);
                }
            }

            double threshold = 1.41;

            return line_distance < threshold;
        }

        return false;
    }
};


struct LinearExpansion
{
    LinearExpansion()
    {
    }

    template <class T, class Map>
    bool expand(const T* start, const T* goal, const Map* map_ptr) {
        start_ = *start;
        goal_ = *goal;

        bresenham.setGrid(map_ptr, std::floor(start_.x), std::floor(start_.y), std::floor(goal_.x),std::floor(goal_.y));

        double theta = std::atan2(goal_.y-start_.y,goal_.x-start_.x);

        unsigned x,y;
        while(bresenham.next()) {
            bresenham.coordinates(x,y);
            if(!map_ptr->isFree(x,y,theta)) {
                return false;
            }
        }

        return true;
    }

    template <class T, class Map>
    static bool canExpand(const T* start, const T* goal, const Map* map_ptr)
    {
        return instance().expand(start,goal, map_ptr);
    }

    static LinearExpansion& instance()
    {
        static LinearExpansion inst;
        return inst;
    }

    template <class PathT>
    void get(PathT* out)
    {
        typedef typename PathT::value_type NodeT;
        NodeT node;
        NodeT::init(node, goal_.x, goal_.y);
        out->push_back(node);
    }

    template <class PathT>
    static void getPath(PathT* out)
    {
        instance().get(out);
    }

    Bresenham2d bresenham;
    Pose2d start_;
    Pose2d goal_;
};



template <typename Algorithm>
struct MapGoalTest
{
    MapGoalTest(Algorithm& algo, const Pose2d& start, double min_dist,
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

        if(val > 32) {
            if(has_heuristic_goal) {
                double dist_to_heuristic_goal = std::hypot(wx - heuristic_goal.x, wy - heuristic_goal.y);
                algo.addGoalCandidate(node, dist_to_heuristic_goal);

            } else {
                algo.addGoalCandidate(node, 255 - val);
            }
            ++candidates;
        }

        return candidates > 64;
    }

    const Pose2d* getHeuristicGoal() const
    {
        if(has_heuristic_goal) {
            std::cerr << "heuristic goal " << &heuristic_goal << std::endl;
            return &heuristic_goal;
        } else {
            std::cerr << "no heuristic goal " << std::endl;
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

    //typedef NonHolonomicNeighborhoodPrecise<70, 240> NHNeighbor;
    //typedef NonHolonomicNeighborhoodPrecise<35, 120> NHNeighbor;
    typedef NonHolonomicNeighborhoodPrecise<40, 120> NHNeighbor;
    typedef NonHolonomicNeighborhoodNoEndOrientation<120, 200> NHNeighborNoEndOrientation;


    DEFINE_CONCRETE_ALGORITHM(AStarNoOrientation,
                              Pose2d, GridMap2d, NHNeighbor, NoExpansion,
                              HeuristicL2, DirectionalStateSpaceManager, PriorityQueueManager);

    //  TODO: make these two (or more?) selectable:
    //typedef AStarNoOrientationSearch<> AStar;
    //    typedef AStarSearch<NonHolonomicNeighborhood<40, 360, NonHolonomicNeighborhoodMoves::FORWARD/*_BACKWARD*/> > AStarAckermann; // Ackermann
    //typedef AStarSearch<NonHolonomicNeighborhoodPrecise<40, 250, NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD>,
    //                    NoExpansion, Pose2d, GridMap2d, 1000> AStarAckermann; // Ackermann
    typedef AStarSearch<NonHolonomicNeighborhoodPrecise<40, 250, NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
    NoExpansion, Pose2d, GridMap2d, 1000> AStarAckermannReversed; // Ackermann
    typedef AStarSearch<NonHolonomicNeighborhoodPrecise<30, 600, NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSummitReversed; // Summit
    typedef AStarSearch<NonHolonomicNeighborhoodPrecise<40, 300, NonHolonomicNeighborhoodMoves::FORWARD, false>,
    NoExpansion, Pose2d, GridMap2d, 100> AStarSummitForward; // Summit Only Forward

    //    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100, true, true> > AStarAckermannRS;
    //    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100, true, false> > AStarAckermannRSForward;
    typedef AStarSearch<NonHolonomicNeighborhood<100, 500, NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>, NoExpansion, Pose2d, GridMap2d, 500 > AStarPatsy;
    typedef AStarSearch<NonHolonomicNeighborhoodPrecise<50, 250, NonHolonomicNeighborhoodMoves::FORWARD, true>, NoExpansion, Pose2d, GridMap2d, 500 > AStarPatsyForward;
    typedef AStarSearch<NonHolonomicNeighborhood<40, 350, NonHolonomicNeighborhoodMoves::FORWARD>,
    ReedsSheppExpansion<100, true, false> > AStarPatsyRSForward;
    //    typedef AStar2dSearch<DirectNeighborhood<8, 1> > AStarOmnidrive; // Omnidrive

    typedef AStarAckermannReversed AStarAckermann;
    typedef AStarSummitReversed AStarSummit;
    //    typedef AStarOmnidrive AStar;

    typedef AStarAckermann::PathT PathT;

    enum class Algo {
        ACKERMANN,
        SUMMIT,
        PATSY,
        PATSY_FORWARD,
        SUMMIT_FORWARD
    };

    PathPlanner()
        : render_open_cells_(false)
    {
        nh_priv.param("render_open_cells", render_open_cells_, false);

        std::string algo = "ackermann";
        nh_priv.param("algorithm", algo, algo);

        nh_priv.param("oversearch_distance", oversearch_distance_, 0.5);

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
        }

        throw std::runtime_error(std::string("unknown algorithm ") + algo);
    }

    nav_msgs::Path path2msg(const PathT& path, const ros::Time &goal_timestamp)
    {
        nav_msgs::Path path_out;
        // set timestamp of the received goal for the path message, so they can be associated
        path_out.header.stamp = goal_timestamp;
        path_out.header.frame_id = "/map";

        if(path.size() >= 2) {
            for(const Pose2d& next_map : path) {
                geometry_msgs::PoseStamped pose;
                map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                             pose.pose.position.y);

                pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

                path_out.poses.push_back(pose);
            }
        }

        return path_out;
    }

    template <typename Algorithm>
    void initSearch (Algorithm& algo,
                     const std_msgs::Header &header)
    {
        algo.setMap(map_info);

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
    nav_msgs::Path planInstance (Algo algo_id, Algorithm& algo,
                                 const path_msgs::PlanPathGoal &request,
                                 const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                                 const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        initSearch(algo, request.goal.pose.header);

        try {
            PathT path;
            if(render_open_cells_) {
                path = algo.findPath(from_map, to_map,
                                     boost::bind(&PathPlanner::renderCells, this, algo_id),
                                     oversearch_distance_);

                // render cells once more -> remove the last ones
                renderCells(algo_id);
            } else {
                path = algo.findPath(from_map, to_map, oversearch_distance_);
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
    nav_msgs::Path planMapInstance (Algo algo_id, Algorithm& algo,
                                    const path_msgs::PlanPathGoal &request,
                                    const Pose2d &from_world, const Pose2d &from_map) {

        initSearch(algo, request.goal.map.header);

        try {

            PathT path;
            MapGoalTest<Algorithm> goal_test(algo, from_world,
                                             request.goal.min_dist,
                                             request.goal.map, map_info);

            if(request.goal.has_search_dir) {
                Pose2d goal;
                goal.x = request.goal.search_dir.x;
                goal.y = request.goal.search_dir.y;
                goal.theta = 0;
                goal_test.setHeuristicGoal(goal);
            }

            if(render_open_cells_) {
                path = algo.findPath(from_map, goal_test,
                                     boost::bind(&PathPlanner::renderCells, this, algo_id),
                                     oversearch_distance_);

                // render cells once more -> remove the last ones
                renderCells(algo_id);
            } else {
                path = algo.findPath(from_map, goal_test, oversearch_distance_);
            }

            return path2msg(path, ros::Time::now());
            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }
        catch(const std::logic_error& e) {
            ROS_ERROR_STREAM("no path found: " << e.what());
        }
        return empty();
    }

    nav_msgs::Path planWithoutTargetPose (const path_msgs::PlanPathGoal &request,
                                          const Pose2d &from_world, const Pose2d &from_map) {

        Algo algorithm = algo_to_use;

        if(!request.goal.algorithm.data.empty()) {
            algorithm = stringToAlgorithm(request.goal.algorithm.data);
        }

        switch(algorithm) {
        case Algo::ACKERMANN:
            return planMapInstance(algorithm, algo_ackermann, request, from_world, from_map);
        case Algo::SUMMIT:
            return planMapInstance(algorithm, algo_summit, request, from_world, from_map);
        case Algo::SUMMIT_FORWARD:
            return planMapInstance(algorithm, algo_summit_forward, request, from_world, from_map);

        default:
            throw std::runtime_error("unknown algorithm selected");
        }

    }

    nav_msgs::Path plan (const path_msgs::PlanPathGoal &goal,
                         const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                         const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        Algo algorithm = algo_to_use;

        if(!goal.goal.algorithm.data.empty()) {
            algorithm = stringToAlgorithm(goal.goal.algorithm.data);
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

        default:
            throw std::runtime_error("unknown algorithm selected");
        }

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

        default:
            throw std::runtime_error("unknown algorithm selected");
        }
    }


private:
    AStarAckermann algo_ackermann;
    AStarSummit algo_summit;
    AStarPatsy algo_patsy;
    AStarPatsyForward algo_patsy_forward;
    AStarSummitForward algo_summit_forward;

    Algo algo_to_use;
    double oversearch_distance_;

    bool render_open_cells_;
    nav_msgs::GridCells cells;
    ros::Publisher cell_publisher_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    PathPlanner planner;

    ros::WallRate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

#endif // PATH_PLANNER_NODE_HPP
