/*
 * path_planner_node.hpp
 *
 *  Created on: Apr 3, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef PATH_PLANNER_NODE_HPP
#define PATH_PLANNER_NODE_HPP

/// PROJECT
#include <utils/LibPath/generic/Algorithms.hpp>
#include <utils/LibPath/common/SimpleGridMap2d.h>
#include <utils/LibPath/evaluation/PathRenderer.hpp>
#include <utils/LibPath/evaluation/MapRenderer.hpp>


/// SYSTEM
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

using namespace lib_path;

struct Planner
{
    enum { SCALE = 1 };

    typedef NonHolonomicNeighborhood<50, 120> NHNeighbor;
//    typedef AStarSearch_Debug<10000, NoSubParameter, MapRenderer, Pose2d, GridMap2d, NHNeighbor > AStar;
    typedef AStarSearch_Debug<10000, NoSubParameter, MapManagerExtension, Pose2d, GridMap2d, NHNeighbor > AStar;


//    typedef PathRenderer<SCALE, AStar::NodeT, AStar::PathT, AStar::Heuristic> Renderer;

    //    typedef DirectNeighborhood<8,5> DNeighbor;
    //    typedef AStar2dSearch_Debug<80, NoSubParameter, MapManagerExtension, Pose2d, GridMap2d, DNeighbor> AStar;

    Planner()
        : nh("~"), map_info(NULL)
    {
        std::string target_topic = "/move_base_simple/goal";
        nh.param("target_topic", target_topic, target_topic);

        goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (target_topic, 2, boost::bind(&Planner::updateGoal, this, _1));

        nh.param("use_map_topic", use_map_topic_, false);
        use_map_service_ = !use_map_topic_;

        if(use_map_topic_) {
            map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
                    ("/map/inflated", 1, boost::bind(&Planner::updateMapCallback, this, _1));

        } else {
            map_service_client = nh.serviceClient<nav_msgs::GetMap>
                    ("/dynamic_map/inflated");
        }


        path_publisher = nh.advertise<nav_msgs::Path> ("path", 10);
        raw_path_publisher = nh.advertise<nav_msgs::Path> ("path_raw", 10);
    }

    void updateMapCallback (const nav_msgs::OccupancyGridConstPtr &map) {
        updateMap(*map);
    }


    void updateMap (const nav_msgs::OccupancyGrid &map) {
        unsigned w = map.info.width;
        unsigned h = map.info.height;

        bool replace = map_info == NULL ||
                map_info->getWidth() != w ||
                map_info->getHeight() != h;

        if(replace){
            if(map_info != NULL) {
                delete map_info;
            }
            map_info = new lib_path::SimpleGridMap2d(map.info.width, map.info.height, map.info.resolution);
        }

        /// Map data
        /// -1: unknown -> 0
        /// 0:100 probabilities -> 1 - 101
        std::vector<uint8_t> data(w*h);
        int i = 0;
        for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
            data[i++] = *it + 1;
        }

        map_info->set(data, w, h);
        map_info->setOrigin(Point2d(map.info.origin.position.x, map.info.origin.position.y));
        map_info->setLowerThreshold(10);
        map_info->setUpperThreshold(70);
    }

    nav_msgs::Path path2msg(const AStar::PathT& path)
    {
        nav_msgs::Path path_out;
        path_out.header.frame_id = "/map";

        const Pose2d* last = NULL;
        BOOST_FOREACH(const Pose2d& next_map, path) {
            geometry_msgs::PoseStamped pose;
            map_info->cell2pointSubPixel(next_map.x,next_map.y,pose.pose.position.x,
                                      pose.pose.position.y);

            pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_map.theta);

            path_out.poses.push_back(pose);
        }

        return path_out;
    }

    void updateGoal (const geometry_msgs::PoseStampedConstPtr &goal) {
        std::cout << "got goal" << std::endl;
        if(use_map_service_) {
            nav_msgs::GetMap map_service;
            map_service_client.call(map_service);
            updateMap(map_service.response.map);
        }

        if(map_info == NULL) {
            ROS_WARN("request for path planning, but no map there yet...");
            return;
        }

        std::cout << "starting search" << std::endl;
        lib_path::Pose2d from_world;
        lib_path::Pose2d to_world;

        tf::StampedTransform trafo;
        tfl.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), trafo);

        from_world.x = trafo.getOrigin().x();
        from_world.y = trafo.getOrigin().y();
        from_world.theta = tf::getYaw(trafo.getRotation());

        ROS_WARN_STREAM("theta=" << from_world.theta);

        to_world.x = goal->pose.position.x;
        to_world.y = goal->pose.position.y;
        to_world.theta = tf::getYaw(goal->pose.orientation);

        lib_path::Pose2d from_map;
        lib_path::Pose2d to_map;

        {
            unsigned fx, fy;
            map_info->point2cell(from_world.x, from_world.y, fx, fy);
            from_map.x = fx;
            from_map.y = fy;
            from_map.theta = from_world.theta;
        }

        {
            unsigned fx, fy;
            map_info->point2cell(to_world.x, to_world.y, fx, fy);
            to_map.x = fx;
            to_map.y = fy;
            to_map.theta = to_world.theta;
        }


        algo.setMap(map_info);

        AStar::PathT path = algo.findPath(from_map, to_map);

        if(path.empty()) {
            std::cout << "no path found" << std::endl;
        } else {
            std::cout << "path with " << path.size() << " nodes found" << std::endl;
        }



        AStar::PathT smooted_path = smoothPath(path, 0.5, 0.35);

        /// path
        raw_path_publisher.publish(path2msg(path));
        path_publisher.publish(path2msg(smooted_path));
    }

    AStar::PathT smoothPath(const AStar::PathT& path, double weight_data, double weight_smooth, double tolerance = 0.000001) {
        AStar::PathT new_path = path;

        double last_change = -2 * tolerance;
        double change = 0;

        unsigned n = path.size();

        if(n < 2) {
            return AStar::PathT();
        }

        while(change > last_change + tolerance) {
            last_change = change;
            change = 0;

            for(unsigned i = 1; i < n-1; ++i){
                Pose2d deltaData = weight_data * (path[i] - new_path[i]);
                Pose2d nextData = new_path[i] + deltaData;
                new_path[i].x = nextData.x;
                new_path[i].y = nextData.y;

                Pose2d deltaSmooth =  weight_smooth * (new_path[i+1] + new_path[i-1] - 2* new_path[i]);
                Pose2d nextSmooth = new_path[i] + deltaSmooth;
                new_path[i].x = nextSmooth.x;
                new_path[i].y = nextSmooth.y;

                change += deltaData.distance_to_origin() + deltaSmooth.distance_to_origin();
            }
        }

        // update orientations
        for(unsigned i = 1; i < n-1; ++i){
            Pose2d delta = new_path[i+1] - new_path[i-1];
            new_path[i].theta = std::atan2(delta.y, delta.x);
        }

        return new_path;
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber goal_pose_sub;

    ros::Subscriber map_sub;
    ros::ServiceClient map_service_client;

    ros::Publisher path_publisher;
    ros::Publisher raw_path_publisher;

    bool use_map_topic_;
    bool use_map_service_;

    lib_path::SimpleGridMap2d * map_info;
    AStar algo;

    tf::TransformListener tfl;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    Planner planner;

    ros::WallRate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

#endif // PATH_PLANNER_NODE_HPP
