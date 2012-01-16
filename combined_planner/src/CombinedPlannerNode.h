/**
 * @file CombinedPlannerNode.h
 * @date Jan 2012
 * @author marks
 */

// C/C++
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

// LibPath
#include <utils/LibPath/common/SimpleGridMap2d.h>
#include <utils/LibPath/a_star/AStar.h>

class CombinedPlannerNode {
public:

    CombinedPlannerNode();

    void updateMap( const nav_msgs::OccupancyGridConstPtr& map );
    void updateGoal( const geometry_msgs::PoseStampedConstPtr& goal );
    bool getRobotPose( geometry_msgs::Pose& pose, const std::string& map_frame );
    void visualizeGlobalPath( const std::vector<geometry_msgs::Point> &path );
    bool isFree( const lib_path::waypoint_t p1, const lib_path::waypoint_t p2 );

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
    lib_path::AStar* a_star_;
    bool got_map_;
};
