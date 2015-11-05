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
    static bool isNearEnough(NodeType* goal, NodeType* reference) {
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
    static bool isNearEnough(NodeType* goal, NodeType* reference) {
        double angle_dist_ref = MathHelper::AngleClamp(goal->theta - reference->theta);
        static const double angle_threshold = M_PI / 8;
        if(std::abs(angle_dist_ref) > angle_threshold) {
            return false;
        }

        double dist = hypot(goal->x - reference->x, goal->y - reference->y);


        // euclidean distance
		  int delta = 2;
        if(dist < 5.0) return true;

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
    typedef AStarSearch<NonHolonomicNeighborhoodPrecise<30, 500, NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, true>,
                        NoExpansion, Pose2d, GridMap2d, 100> AStarSummitReversed; // Summit
    //    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100, true, true> > AStarAckermannRS;
    //    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100, true, false> > AStarAckermannRSForward;
    typedef AStarSearch<NonHolonomicNeighborhood<40, 250, NonHolonomicNeighborhoodMoves::FORWARD> > AStarPatsyForward;
    typedef AStarSearch<NonHolonomicNeighborhood<40, 250, NonHolonomicNeighborhoodMoves::FORWARD>,
    ReedsSheppExpansion<100, true, false> > AStarPatsyRSForward;
    //    typedef AStar2dSearch<DirectNeighborhood<8, 1> > AStarOmnidrive; // Omnidrive

    typedef AStarAckermannReversed AStarAckermann;
    typedef AStarSummitReversed AStarSummit;
    //    typedef AStarOmnidrive AStar;

    typedef AStarAckermann::PathT PathT;

    enum class Algo {
        ACKERMANN = 0,
        SUMMIT = 1
    };

    PathPlanner()
        : render_open_cells_(false)
    {
        nh_priv.param("render_open_cells", render_open_cells_, false);

        std::string algo = "ackermann";
        nh_priv.param("algorithm", algo, algo);

        std::transform(algo.begin(), algo.end(), algo.begin(), ::tolower);

        algo_to_use = stringToAlgorithm(algo);

        ROS_INFO_STREAM("planner uses algorithm: " << algo);

        if(render_open_cells_) {
            cell_publisher_ = nh_priv.advertise<nav_msgs::GridCells>("cells", 1, true);
        }
    }

    Algo stringToAlgorithm(const std::string& algo) const
    {
        if(algo == "ackermann") {
            return Algo::ACKERMANN;
        } else if(algo == "summit") {
            return Algo::SUMMIT;
        }

        throw std::runtime_error(std::string("unknown algorithm ") + algo);
    }

    nav_msgs::Path path2msg(const PathT& path, const ros::Time &goal_timestamp)
    {
        nav_msgs::Path path_out;
        // set timestamp of the received goal for the path message, so they can be associated
        path_out.header.stamp = goal_timestamp;
        path_out.header.frame_id = "/map";

        for(const Pose2d& next_map : path) {
            geometry_msgs::PoseStamped pose;
            map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                         pose.pose.position.y);

            pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

            path_out.poses.push_back(pose);
        }

        return path_out;
    }

    //    double getCost(int x, int y)
    //    {
    //        if(x < 0 || y < 0 || x >= (int) cost_map.info.width || y >= (int) cost_map.info.height) {
    //            return std::numeric_limits<double>::max();
    //        }

    //        static const double penalty_distance = 10.5;
    //        double norm_cost = cost_map.data[cost_map.info.width * y + x] / 100.0;
    //        double cost = norm_cost * map_info->getResolution() * penalty_distance;
    //        return cost;
    //    }


    template <typename Algorithm>
    nav_msgs::Path planInstance (Algorithm& algo,
                                 const path_msgs::PlanPathGoal &goal,
                                 const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                                 const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        algo.setMap(map_info);

        if(use_cost_map_) {
            cells.header = cost_map.header;
            cells.cell_width = cells.cell_height = cost_map.info.resolution;
            cells.cells.clear();

            algo.setCostFunction(true);
        }

        PathT path;
        try {
            ROS_INFO_STREAM("planning from " << from_map.theta <<
                            "\nto " << to_map.theta);

            if(render_open_cells_) {
                path = algo.findPath(from_map, to_map, boost::bind(&PathPlanner::renderCells, this));

                // render cells once more -> remove the last ones
                renderCells();
            } else {
                path = algo.findPath(from_map, to_map);
            }

            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }
        catch(const std::logic_error& e) {
            ROS_ERROR_STREAM("no path found: " << e.what());
            path = algo.empty();
        }

        return path2msg(path, goal.goal.header.stamp);

    }

    nav_msgs::Path plan (const path_msgs::PlanPathGoal &goal,
                         const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                         const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        Algo algorithm = algo_to_use;

        if(!goal.algorithm.data.empty()) {
            algorithm = stringToAlgorithm(goal.algorithm.data);
        }

        switch(algorithm) {
        case Algo::ACKERMANN:
            return planInstance(algo_ackermann, goal, from_world, to_world, from_map, to_map);
        case Algo::SUMMIT:
            return planInstance(algo_summit, goal, from_world, to_world, from_map, to_map);

        default:
            throw std::runtime_error("unknown algorithm selected");
        }

    }

    template <class Algorithm>
    void renderCellsInstance(Algorithm& algo)
    {
        if(use_cost_map_ && render_open_cells_) {
            auto& open = algo.getOpenList();

            /*
            double res = cost_map.info.resolution;
            double ox = cost_map.info.origin.position.x;
            double oy = cost_map.info.origin.position.y;
*/
            for(auto it = open.begin(); it != open.end(); ++it) {
                const auto* node = *it;
                geometry_msgs::Point pt;
                map_info->cell2point(node->x, node->y, pt.x, pt.y);
                cells.cells.push_back(pt);
            }


            const auto* node_start = algo.getStart();
            const auto* node_goal = algo.getGoal();

            geometry_msgs::Point pt_start, pt_goal;
            map_info->cell2point(node_start->x, node_start->y, pt_start.x, pt_start.y);
            map_info->cell2point(node_goal->x, node_goal->y, pt_goal.x, pt_goal.y);

            cells.cells.push_back(pt_start);
            cells.cells.push_back(pt_goal);

            std::cerr << "publish cells" << std::endl;

            cell_publisher_.publish(cells);
            cells.cells.clear();
        }
    }

    void renderCells()
    {
        switch(algo_to_use) {
        case Algo::ACKERMANN:
            return renderCellsInstance(algo_ackermann);
        case Algo::SUMMIT:
            return renderCellsInstance(algo_summit);

        default:
            throw std::runtime_error("unknown algorithm selected");
        }
    }


private:
    AStarAckermann algo_ackermann;
    AStarSummit algo_summit;

    Algo algo_to_use;

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
