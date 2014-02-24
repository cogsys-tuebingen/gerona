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
        return std::abs(goal->x - reference->x) <= 1 &&
                std::abs(goal->y - reference->y) <= 1 &&
                std::abs(MathHelper::AngleClamp(goal->theta - reference->theta)) < M_PI / 10;
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
    typedef AStarSearch<NHNeighbor, ReedsSheppExpansion<100, true, false> > AStar;

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

    void plan (const geometry_msgs::PoseStamped &goal,
               const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
               const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {
        algo.setMap(map_info);

        PathT path = algo.findPath(from_map, to_map);

        if(path.empty()) {
            ROS_WARN("no path found");
        } else {
            ROS_INFO_STREAM("path with " << path.size() << " nodes found");
        }

        PathT interpolated_path = interpolatePath(path, 1);
        PathT smooted_path = smoothPath(interpolated_path, 0.5, 0.3);

        /// path
        raw_path_publisher.publish(path2msg(path, goal.header.stamp));
        path_publisher.publish(path2msg(smooted_path, goal.header.stamp));
    }

    void split(PathT& result, PathT::NodeT low, PathT::NodeT up, double max_distance) {
        double distance = low.distance_to(up);
        if(distance > max_distance) {
            // split half way between the lower and the upper node
            PathT::NodeT halfway(low);
            halfway.x += (up.x - low.x) / 2.0;
            halfway.y += (up.y - low.y) / 2.0;
            halfway.theta += (up.theta - low.theta) / 2.0;

            // first recursive descent in lower part
            split(result, low, halfway, max_distance);
            // then add the half way point
            result.push_back(halfway);
            // then descent in upper part
            split(result, halfway, up, max_distance);
        }
    }

    PathT interpolatePath(const PathT& path, double max_distance) {
        unsigned n = path.size();
        if(n < 2) {
            return PathT();
        }

        PathT result;
        result.push_back(path[0]);

        for(int i = 1; i < n; ++i){
            const PathT::NodeT* current = &path[i];

            // split the segment, iff it is to large
            split(result, path[i-1], path[i], max_distance);

            // add the end of the segment (is not done, when splitting)
            result.push_back(*current);
        }
        return result;
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
