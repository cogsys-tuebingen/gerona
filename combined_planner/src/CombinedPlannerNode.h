/**
 * @file CombinedPlannerNode.h
 * @date Jan 2012
 * @author marks
 */

// C/C++
#include <string>
#include <vector>
#include <list>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

// LibPath
#include <utils/LibPath/common/Point2d.h>
#include <utils/LibPath/common/SimpleGridMap2d.h>

// Project
#include "GlobalPlanner.h"

class CombinedPlannerNode {
public:

    CombinedPlannerNode();

    void updateMap( const nav_msgs::OccupancyGridConstPtr& map );
    void updateGoal( const geometry_msgs::PoseStampedConstPtr& goal );
    bool getRobotPose( geometry_msgs::Pose& pose, const std::string& map_frame );
    void visualizePath( const std::list<lib_path::Point2d> &path,
                        const std::string& ns,
                        const int color = 0,
                        const int id = 0 );

    bool selectNextWaypoint( const lib_path::Pose2d& robot_pose,
                             lib_path::Pose2d& next ) const;

private:

    std::string map_topic_;
    std::string goal_topic_;
    std::string path_topic_;

    ros::NodeHandle n_;
    ros::Publisher path_pub_;
    ros::Publisher visu_pub_;
    ros::Subscriber map_subs_;
    ros::Subscriber goal_subs_;
    tf::TransformListener tf_;

    lib_path::SimpleGridMap2d* map_;
    std::string map_frame_id_;
    GlobalPlanner* global_planner_;
    bool got_map_;
    std::list<lib_path::Point2d> global_path_;
};
