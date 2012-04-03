#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <motion_control/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point32.h>
#include "traversable_path/LaserScanClassification.h"

class PathFollower
{
public:
    PathFollower();

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber subscribe_scan_classification_;
    ros::Subscriber subscribe_drive_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionClient<motion_control::MotionAction> motion_control_action_client_;
    //! Last goal. Used to calculate the orientation of the new goal.
    geometry_msgs::Point32 last_goal_;

    void scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan_classification);
    void drive(std_msgs::BoolConstPtr b);
};

#endif // PATHFOLLOWER_H
