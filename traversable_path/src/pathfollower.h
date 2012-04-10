#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <motion_control/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include "traversable_path/LaserScanClassification.h"

class PathFollower
{
public:
    PathFollower();

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber subscribe_scan_classification_;
    ros::Publisher publish_rviz_marker_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionClient<motion_control::MotionAction> motion_control_action_client_;
    //! The current goal.
    geometry_msgs::Point current_goal_;


    void scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan_classification);
    void publishGoalMarker(geometry_msgs::PoseStamped goal);
    void publishTraversaleLineMarker(geometry_msgs::Point32 a, geometry_msgs::Point32 b);
};

#endif // PATHFOLLOWER_H
