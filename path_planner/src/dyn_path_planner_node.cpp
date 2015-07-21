/*
 * dyn_path_planner_node.hpp
 *
 *  Created on: Jun 3, 2015
 *      Author: luis <l.ibargueen-gonzalez@student.uni-tuebingen.de>
 */

#ifndef DYN_PATH_PLANNER_NODE_HPP
#define DYN_PATH_PLANNER_NODE_HPP

/// COMPONENT
#include "planner_node.h"

/// PROJECT
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/generic/ReedsSheppExpansion.hpp>
#include <utils_path/common/Bresenham2d.h>

/// SYSTEM
#include <nav_msgs/Path.h>

using namespace lib_path;


/**
 * @brief The PathPlanner struct uses our custom planning algorithms to plan paths
 */
struct DynPathPlanner : public Planner
{
    //enum { SCALE = 1 };
    typedef DStar2DSearch<DirectNeighborhood<8, 1> > DStar;
    typedef DStar::PathT PathT;

    DynPathPlanner()
    {}

    nav_msgs::Path path2msg(const PathT& path, const ros::Time &goal_timestamp)
    {
        nav_msgs::Path path_out;
        // set timestamp of the received goal for the path message, so they can be associated
        //path_out.header.stamp = goal_timestamp;
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

        static const double penalty_distance = 10.5;
        double norm_cost = cost_map.data[cost_map.info.width * y + x] / 100.0;
        double cost = norm_cost * map_info->getResolution() * penalty_distance;
        return cost;
    }

    nav_msgs::Path plan (const geometry_msgs::PoseStamped &goal,
                         const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                         const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {
        algo.setMap(map_info);
        PathT path;
        try {
            ROS_INFO_STREAM("planning from " << from_map.theta <<
                            "\nto " << to_map.theta);
            algo.init(from_map, to_map);
            path = algo.findPath();
            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }
        catch(const std::logic_error& e) {
            ROS_ERROR_STREAM("no path found: " << e.what());
            path = algo.empty();
        }

        return path2msg(path, goal.header.stamp);


//        nav_msgs::Path p;
//        geometry_msgs::PoseStamped start,ziel;
//        start.pose.position.x = from_world.x;
//        start.pose.position.y = from_world.y;
//        ziel.pose.position.x = to_world.x;
//        ziel.pose.position.y = to_world.y;
//        p.header.frame_id = "/map";
//        p.poses.push_back(start);
//        p.poses.push_back(ziel);
//        return p;
    }


private:
    DStar algo;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dyn_path_planner");

    DynPathPlanner planner;

    ros::WallRate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

#endif // DYN_PATH_PLANNER_NODE_HPP
