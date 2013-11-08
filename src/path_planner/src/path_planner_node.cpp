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
#include <utils/LibPath/generic/ReedsSheppExpansion.hpp>
#include <utils/LibPath/common/SimpleGridMap2d.h>


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


struct Planner
{
    enum { SCALE = 1 };

    typedef NonHolonomicNeighborhood<50, 120> NHNeighbor;
    typedef NonHolonomicNeighborhoodNoEndOrientation<120, 200> NHNeighborNoEndOrientation;


    DEFINE_CONCRETE_ALGORITHM(AStarNoOrientation,
                              Pose2d, GridMap2d, NHNeighborNoEndOrientation, NoExpansion,
                              HeuristicL2, DirectionalStateSpaceManager, PriorityQueueManager)

//  TODO: make these two (or more?) selectable:
    typedef AStarNoOrientationSearch<> AStar;
//    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100> > AStar;

    typedef AStar::PathT PathT;

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
          std::string map_topic = "/map/inflated";
          nh.param("map_topic",map_topic, map_topic);
          map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
                   (map_topic, 1, boost::bind(&Planner::updateMapCallback, this, _1));

        } else {
            map_service_client = nh.serviceClient<nav_msgs::GetMap>
                    ("/dynamic_map/inflated");
        }

        base_frame_ = "/base_link";
        nh.param("base_frame", base_frame_, base_frame_);


        path_publisher = nh.advertise<nav_msgs::Path> ("/path", 10);
        raw_path_publisher = nh.advertise<nav_msgs::Path> ("/path_raw", 10);
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

    nav_msgs::Path path2msg(const PathT& path)
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
            if(map_service_client.call(map_service)) {
                updateMap(map_service.response.map);
            }
        }

        if(map_info == NULL) {
            ROS_WARN("request for path planning, but no map there yet...");
            return;
        }

        std::cout << "starting search" << std::endl;
        lib_path::Pose2d from_world;
        lib_path::Pose2d to_world;

        tf::StampedTransform trafo;
        tfl.lookupTransform("/map", base_frame_, ros::Time(0), trafo);

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

        ROS_WARN_STREAM("res=" << map_info->getResolution());
        {
            unsigned tx, ty;
            map_info->point2cell(to_world.x, to_world.y, tx, ty);
            to_map.x = tx;
            to_map.y = ty;
            to_map.theta = to_world.theta;
        }


        algo.setMap(map_info);

        PathT path = algo.findPath(from_map, to_map);

        if(path.empty()) {
            std::cout << "no path found" << std::endl;
        } else {
            std::cout << "path with " << path.size() << " nodes found" << std::endl;
        }



        PathT smooted_path = smoothPath(path, 0.5, 0.1);

        /// path
        raw_path_publisher.publish(path2msg(path));
        path_publisher.publish(path2msg(smooted_path));
    }

    PathT smoothPath(const PathT& path, double weight_data, double weight_smooth, double tolerance = 0.000001) {
        // find segments
        unsigned n = path.size();
        if(n < 2) {
            return PathT();
        }

        PathT result;
        PathT current_segment;

        const PathT::NodeT * last_point = &path[0];
        current_segment.push_back(*last_point);

        for(int i = 1; i < n; ++i){
            const PathT::NodeT* current_point = &path[i];

            // append to current segment
            current_segment.push_back(*current_point);

            bool is_the_last_node = i == n-1;
            bool segment_ends_with_this_node = false;

            if(is_the_last_node) {
                // this is the last node
                segment_ends_with_this_node = true;

            } else {
                const PathT::NodeT* next_point = &path[i+1];

                // if angle between last direction and next direction to large -> segment ends
                Pose2d diff_last = (*current_point - *last_point);
                double last_angle = std::atan2(diff_last.y, diff_last.x);

                Pose2d diff_next = (*next_point - *current_point);
                double next_angle = std::atan2(diff_next.y, diff_next.x);

                if(std::abs(MathHelper::AngleClamp(last_angle - next_angle)) > M_PI / 2.0) {
                    // new segment!
                    // current node is the last one of the old segment
                    segment_ends_with_this_node = true;
                }
            }

            if(segment_ends_with_this_node) {
                PathT smoothed_segment = smoothPathSegment(current_segment, weight_data, weight_smooth, tolerance);
                result += smoothed_segment;

                current_segment.clear();

                if(!is_the_last_node) {
                    // begin new segment

                    // current node is also the first one of the new segment
                    current_segment.push_back(*current_point);
                }
            }

            last_point = current_point;
        }

        return result;
    }

    PathT smoothPathSegment(const PathT& path, double weight_data, double weight_smooth, double tolerance) {
        PathT new_path = path;

        unsigned n = path.size();
        if(n < 2) {
            return new_path;
        }

        double last_change = -2 * tolerance;
        double change = 0;

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

            if(!new_path[i].forward) {
                new_path[i].theta = MathHelper::AngleClamp(new_path[i].theta + M_PI);
            }
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
    std::string base_frame_;
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
