#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <motion_control/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>

#include "point_types.h"

/**
 * @brief Main class of the follow_path node.
 *
 * The class subscribes for the terrain classification of the node classify_path, determines goal points for navigation
 * and commands the robot to drive to this goals (using the motion_control package).
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class PathFollower
{
public:
    PathFollower();

private:
    ros::NodeHandle node_handle_;
    //! Subscriber for the terrain classification of the node classify_path.
    ros::Subscriber subscribe_scan_classification_;
    //! Publisher for rvis markers.
    ros::Publisher publish_rviz_marker_;
    ros::Publisher publish_goal_;
    //! Listener for tf data.
    tf::TransformListener tf_listener_;
    //! Sends commands to motion_control
    actionlib::SimpleActionClient<motion_control::MotionAction> motion_control_action_client_;

    //! The current goal.
    geometry_msgs::Point current_goal_;


    /**
     * @brief Chooses traversable segments on the path and determines a goal.
     *
     * This is the callback function for the terrain classification messages. It chooses the traversable segment, that
     * lies most in the mid of the scan (and thus should be straight in front of the robot) and determines goal points
     * to drive in the middle of the path.
     * The goals are send to motion_control so this method does the driving.
     *
     * @param The terrain classification of the current laser scan.
     */
    void scan_classification_callback(const pcl::PointCloud<PointXYZRGBT>::ConstPtr& scan_classification);

    /**
     * @brief Sends a marker to rviz which visualizes the goal as an arrow.
     * The arrow starts at the goal position and points in the direction of the goal orientation.
     * @param goal The goal pose with position and orientation of the goal.
     */
    void publishGoalMarker(const geometry_msgs::PoseStamped &goal);

    /**
     * @brief Sends a marker to rviz which visualizes the choosen traversable segment.
     * The traversable segment will be displayed as a green line that connects the left and the right border of the
     * segment.
     * @param a Left border of the traversable segment.
     * @param b Right border of the traversable segment.
     * @param header Header information of the points a and b.
     */
    void publishTraversaleLineMarker(PointXYZRGBT a, PointXYZRGBT b, std_msgs::Header header);
};

#endif // PATHFOLLOWER_H
