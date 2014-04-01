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

/// SYSTEM
#include <nav_msgs/Path.h>

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
template <int n, int distance>
struct NonHolonomicNeighborhoodPrecise :
        public NonHolonomicNeighborhood<n, distance> {
    typedef NonHolonomicNeighborhood<n, distance> Parent;

    using Parent::distance_step_pixel;

    template <class NodeType>
    static bool isNearEnough(NodeType* goal, NodeType* reference) {
        double angle_dist_ref = MathHelper::AngleClamp(goal->theta - reference->theta);
        static const double angle_threshold = M_PI / 32;
        if(std::abs(angle_dist_ref) > angle_threshold) {
            return false;
        }


        // euclidean distance
        int delta = 8;
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
//            std::cerr << "distance: " << line_distance << ", okay?" << (line_distance < threshold) << std::endl;

            return line_distance < threshold;
        }

        return false;
    }
};

/**
 * @brief The PathPlanner struct uses our custom planning algorithms to plan paths
 */
struct PathPlanner : public Planner
{
    enum { SCALE = 1 };

    //typedef NonHolonomicNeighborhoodPrecise<70, 240> NHNeighbor;
    typedef NonHolonomicNeighborhoodPrecise<35, 120> NHNeighbor;
    typedef NonHolonomicNeighborhoodNoEndOrientation<120, 200> NHNeighborNoEndOrientation;


    DEFINE_CONCRETE_ALGORITHM(AStarNoOrientation,
                              Pose2d, GridMap2d, NHNeighbor, NoExpansion,
                              HeuristicL2, DirectionalStateSpaceManager, PriorityQueueManager)

    //  TODO: make these two (or more?) selectable:
    //typedef AStarNoOrientationSearch<> AStar;
    //    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100> > AStar;
    typedef AStarSearch<NHNeighbor/*, ReedsSheppExpansion<100, true, false>*/ > AStar;

    typedef AStar::PathT PathT;

    PathPlanner()
    {
    }

    nav_msgs::Path path2msg(const PathT& path, const ros::Time &goal_timestamp)
    {
        nav_msgs::Path path_out;
        // set timestamp of the received goal for the path message, so they can be associated
        path_out.header.stamp = goal_timestamp;
        path_out.header.frame_id = "/map";

        BOOST_FOREACH(const Pose2d& next_map, path) {
            geometry_msgs::PoseStamped pose;
            map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                         pose.pose.position.y);

            pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

            path_out.poses.push_back(pose);
        }

        return path_out;
    }

    double getCost(int x, int y)
    {
        if(x < 0 || y < 0 || x >= (int) cost_map.info.width || y >= (int) cost_map.info.height) {
            return std::numeric_limits<double>::max();
        }

        static const double penalty_distance = 2.5;
        return cost_map.data[cost_map.info.width * y + x] / 100.0 * map_info->getResolution() * penalty_distance;
    }

    nav_msgs::Path plan (const geometry_msgs::PoseStamped &goal,
               const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
               const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {
        algo.setMap(map_info);

        if(use_cost_map_) {
            algo.setCostFunction(boost::bind(&PathPlanner::getCost, this, _1, _2));
        }

        PathT path = algo.findPath(from_map, to_map);

        if(path.empty()) {
            ROS_WARN("no path found");
        } else {
            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }

        return path2msg(path, goal.header.stamp);
    }


private:
    AStar algo;
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
