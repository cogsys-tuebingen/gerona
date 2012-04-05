#include "pathfollower.h"

PathFollower::PathFollower() :
        motion_control_action_client_("motion_control")
{
    subscribe_scan_classification_ = node_handle_.subscribe("/scan/traversability", 100,
                                                            &PathFollower::scan_classification_callback, this);
    subscribe_drive_ = node_handle_.subscribe("/go", 1, &PathFollower::drive, this);
    publish_rviz_marker_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void PathFollower::scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan)
{
    int goal_index;

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

    // goal = point in the middle of the path
    goal_index = beginning + (end-beginning)/2;

    // transform goal-pose to map-frame
    geometry_msgs::PoseStamped goal_point_laser;
    geometry_msgs::PoseStamped goal_point_map;

    goal_point_laser.header.frame_id = "/laser";
    goal_point_laser.header.stamp = ros::Time(0);
    goal_point_laser.pose.position.x = scan->points[goal_index].x;
    goal_point_laser.pose.position.y = scan->points[goal_index].y;
    //goal_point_laser.pose.position.z = scan->points[goal_index].z;
    goal_point_laser.pose.position.z = 0;

//    // dont change orientation for the moment.
//    /** @todo in the future, this should look along the path */
//    goal_point_laser.pose.orientation.x = 0;
//    goal_point_laser.pose.orientation.y = 0;
//    goal_point_laser.pose.orientation.z = 0;
//    goal_point_laser.pose.orientation.w = 1;

    // orientation: robot to goal
    double theta = atan2(goal_point_laser.pose.position.y, goal_point_laser.pose.position.x);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta), goal_point_laser.pose.orientation);


    try {
        tf_listener_.transformPose("/odom", goal_point_laser, goal_point_map);
    }
    catch (tf::TransformException e) {
        ROS_WARN("Unable to transform goal. tf says: %s", e.what());
        return;
        /** @todo stop robot? */
    }


    // send goal to motion_control
    motion_control::MotionGoal goal;
    goal.v     = 0.2;
    goal.beta  = 0;
    goal.mode  = motion_control::MotionGoal::MOTION_TO_GOAL;

    goal.x     = goal_point_map.pose.position.x;
    goal.y     = goal_point_map.pose.position.y;

    /* Orientation
     * Look along the line from the last goal to the new one.
     */
//    if (last_goal_.x == 0 && last_goal_.y == 0) {
//        goal.theta = tf::getYaw(goal_point_map.pose.orientation);
//    } else {
//        ROS_INFO("oldgoal");
//        goal.theta = atan2(goal.y - last_goal_.y, goal.x - last_goal_.y);
//    }
//
//    // only change last goal, if distance is big enough
//    float distance = sqrt( pow(last_goal_.x - goal.x, 2) + pow(last_goal_.y - goal.y, 2) );
//    if (distance > 0.1) {
//        last_goal_.x = goal.x;
//        last_goal_.y = goal.y;
//    }
    goal.theta = tf::getYaw(goal_point_map.pose.orientation);

    // send goal to motion_control
    motion_control_action_client_.sendGoal(goal);

    // send goal-marker to rviz for debugging
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal.theta), goal_point_map.pose.orientation);
    publishGoalMarker(goal_point_map);


//    ROS_INFO("Goal (laser): x: %f; y: %f; theta: %f;; x: %f, y: %f, z: %f, w: %f", goal_point_laser.pose.position.x,
//             goal_point_laser.pose.position.y, tf::getYaw(goal_point_laser.pose.orientation),
//             goal_point_laser.pose.orientation.x,
//             goal_point_laser.pose.orientation.y,
//             goal_point_laser.pose.orientation.z,
//             goal_point_laser.pose.orientation.w);
//    ROS_INFO("Goal (map): x: %f; y: %f; theta: %f;; x: %f, y: %f, z: %f, w: %f", goal_point_map.pose.position.x,
//             goal_point_map.pose.position.y, tf::getYaw(goal_point_map.pose.orientation),
//             goal_point_map.pose.orientation.x,
//             goal_point_map.pose.orientation.y,
//             goal_point_map.pose.orientation.z,
//             goal_point_map.pose.orientation.w);
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
        tf_listener_.transformPose("/odom",ros::Time(0), path_point,"/base_link", goal_point);
        ROS_INFO("current position: %f,%f", goal_point.pose.position.x, goal_point.pose.position.y);

        path_point.pose.position.x = 2;
        path_point.pose.position.y = 0;
        tf_listener_.transformPose("/odom",ros::Time(0), path_point,"/base_link", goal_point);
        ROS_INFO("goal position: %f,%f", goal_point.pose.position.x, goal_point.pose.position.y);

        motion_control::MotionGoal goal;
        goal.v=0.4;
        goal.beta=0;
        goal.x = goal_point.pose.position.x;
        goal.y = goal_point.pose.position.y;
        goal.theta = 0;
        goal.mode=motion_control::MotionGoal::MOTION_TO_GOAL;

        ROS_INFO("Start");
        motion_control_action_client_.sendGoal(goal);
    }
    catch (tf::InvalidArgument e) {
        ROS_ERROR("Problem: %s", e.what());
    }
}

void PathFollower::publishGoalMarker(geometry_msgs::PoseStamped goal)
{
    visualization_msgs::Marker marker;

    marker.header = goal.header;
    marker.pose   = goal.pose;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "follow_path";
    marker.id = 0;

    // Set the marker type.
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    publish_rviz_marker_.publish(marker);
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
