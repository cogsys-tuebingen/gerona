#include "pathfollower.h"

PathFollower::PathFollower() :
        action_client_("motion_control")
{
    subscribe_scan_classification_ = node_handle_.subscribe("/scan/traversability", 100,
                                                            &PathFollower::scan_classification_callback, this);
    subscribe_drive_ = node_handle_.subscribe("/go", 1, &PathFollower::drive, this);
}

void PathFollower::scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan)
{
    int path;

    // search traversable area in front of the robot (assuming, "in front" is approximalty in the middle of the scan)
    unsigned int mid = scan->traversable.size() / 2;


//    //tf::StampedTransform transform;
//    geometry_msgs::PointStamped laser_point;
//    try {
//        //tf_listener_.lookupTransform("/base_link", "/laser", ros::Time(0), transform);
//        tf_listener_.transformPoint();
//        ROS_INFO("%f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
//    }
//    catch (tf::TransformException ex) {
//        ROS_ERROR("%s",ex.what());
//    }
//
//    return;


    // search traversable area
    unsigned int beginning = 0, end = 0;
    for (unsigned int i = 0; i < mid; ++i) {
        if (scan->traversable[mid+i]) {
            beginning = mid+i;
            end = mid+i;
            break;
        } else if (scan->traversable[mid-i]) {
            beginning = mid-i;
            end = mid-i;
            break;
        }
    }

    if (beginning == 0) {
        ROS_DEBUG("No traversable paths found.");
        // TODO: stop robot
        return;
    }

    // get range of the area
    while (beginning > 0 && scan->traversable[beginning]) {
        --beginning;
    }
    while (end < scan->traversable.size() && scan->traversable[end]) {
        ++end;
    }

    // path = point in the middle of the path
    path = beginning + (end-beginning)/2;

//    ROS_INFO("size trav: %d, points: %d", scan->traversable.size(), scan->points.size());
//
//    // transform this point to odom-frame
//    geometry_msgs::PointStamped path_point;
//    geometry_msgs::PointStamped goal_point;
//
//    path_point.header.frame_id = "/laser";
//    path_point.header.stamp = ros::Time();
//    path_point.point.x = scan->points[path].x;
//    path_point.point.y = scan->points[path].y;
//    path_point.point.z = scan->points[path].z;
//
//    tf_listener_.transformPoint("/odom", path_point, goal_point);
//
//    ROS_INFO("%f, %f, %f", goal_point.point.x, goal_point.point.y, goal_point.point.z);
}


void PathFollower::drive(std_msgs::BoolConstPtr b) {
    // transform this point to odom-frame
    geometry_msgs::PoseStamped path_point;
    geometry_msgs::PoseStamped goal_point;

    path_point.header.frame_id = "/base_link";
    path_point.header.stamp = ros::Time();
    path_point.pose.position.x = 0;
    path_point.pose.position.y = 0;
    path_point.pose.position.z = 0;
    path_point.pose.orientation.x = 0;
    path_point.pose.orientation.y = 0;
    path_point.pose.orientation.z = 0;
    path_point.pose.orientation.w = 1;

    try {
        tf_listener_.transformPose("odom",ros::Time(0), path_point,"/base_link", goal_point);
        ROS_INFO("current position: %f,%f", goal_point.pose.position.x, goal_point.pose.position.y);

         path_point.pose.position.x = 2;
        tf_listener_.transformPose("odom",ros::Time(0), path_point,"/base_link", goal_point);
        ROS_INFO("goal position: %f,%f", goal_point.pose.position.x, goal_point.pose.position.y);

        motion_control::MotionGoal goal;
        goal.v=0.4;
        goal.beta=0;
        goal.x = goal_point.pose.position.x;
        goal.y = goal_point.pose.position.y;
        goal.theta = 0;
        goal.mode=motion_control::MotionGoal::MOTION_TO_GOAL;

        ROS_INFO("Start");
        action_client_.sendGoal(goal);
    }
    catch (tf::InvalidArgument e) {
        ROS_ERROR("Problem: %s", e.what());
    }
}


//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");

    PathFollower follower;

    // main loop
    ros::spin();
    return 0;
}
