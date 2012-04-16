#include "pathfollower.h"

PathFollower::PathFollower() :
        motion_control_action_client_("motion_control")
{
    subscribe_scan_classification_ = node_handle_.subscribe("traversability", 100,
                                                            &PathFollower::scan_classification_callback, this);
    publish_rviz_marker_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void PathFollower::scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan)
{
    int goal_index;

    // search traversable area in front of the robot (assuming, "in front" is approximalty in the middle of the scan)
    unsigned int mid = scan->traversable.size() / 2;

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
    while (end < scan->traversable.size()-1 && scan->traversable[end]) {
        ++end;
    }

    ROS_DEBUG("size points: %zu, size traversable: %zu, beginning: %d, end: %d",
              scan->points.size(), scan->traversable.size(), beginning, end);

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

    // orientation: orthogonal to the line of the traversable segment
    double delta_x = scan->points[beginning].x - scan->points[end].x;
    double delta_y = scan->points[end].y - scan->points[beginning].y;
    double theta = M_PI/2 - atan2(delta_y, delta_x);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta), goal_point_laser.pose.orientation);
    //ROS_INFO("dx = %f, dy = %f, atan2 = %f, theta = %f", delta_x, delta_y, atan2(delta_y, delta_x), theta);
    publishTraversaleLineMarker(scan->points[beginning], scan->points[end]);


    try {
        tf_listener_.transformPose("/map", goal_point_laser, goal_point_map);
    }
    catch (tf::TransformException e) {
        ROS_WARN("Unable to transform goal. tf says: %s", e.what());
        return;
        /** @todo stop robot? */
    }

    const double MIN_DISTANCE_BETWEEN_GOALS = 0.5;
    double distance = sqrt( pow(goal_point_map.pose.position.x - current_goal_.x, 2) +
                            pow(goal_point_map.pose.position.y - current_goal_.y, 2) );

    if (distance > MIN_DISTANCE_BETWEEN_GOALS) {
        // send goal to motion_control
        motion_control::MotionGoal goal;
        goal.v     = 0.4;
        goal.beta  = 0;
        goal.mode  = motion_control::MotionGoal::MOTION_TO_GOAL;

        goal.x     = goal_point_map.pose.position.x;
        goal.y     = goal_point_map.pose.position.y;

        /* Orientation
         * Look along the line from the last goal to the new one.
         */
        goal.theta = tf::getYaw(goal_point_map.pose.orientation);

        // send goal to motion_control
        motion_control_action_client_.sendGoal(goal);
        current_goal_ = goal_point_map.pose.position;

        // send goal-marker to rviz for debugging
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal.theta), goal_point_map.pose.orientation);
        publishGoalMarker(goal_point_map);


    //    ROS_INFO("Goal (laser): x: %f; y: %f; theta: %f;; x: %f, y: %f, z: %f, w: %f", goal_point_laser.pose.position.x,
    //             goal_point_laser.pose.position.y, tf::getYaw(goal_point_laser.pose.orientation),
    //             goal_point_laser.pose.orientation.x,
    //             goal_point_laser.pose.orientation.y,
    //             goal_point_laser.pose.orientation.z,
    //             goal_point_laser.pose.orientation.w);
        ROS_DEBUG("Goal (map): x: %f; y: %f; theta: %f;; x: %f, y: %f, z: %f, w: %f", goal_point_map.pose.position.x,
                 goal_point_map.pose.position.y, tf::getYaw(goal_point_map.pose.orientation),
                 goal_point_map.pose.orientation.x,
                 goal_point_map.pose.orientation.y,
                 goal_point_map.pose.orientation.z,
                 goal_point_map.pose.orientation.w);
    }
    else {
        ROS_DEBUG("Don't update goal. New goal is %.2f m distant from the current goal. Minimum distance is %f",
                  distance, MIN_DISTANCE_BETWEEN_GOALS);
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

void PathFollower::publishTraversaleLineMarker(geometry_msgs::Point32 a, geometry_msgs::Point32 b)
{
    //ROS_INFO("mark line from point a(%f,%f,%f) to b(%f,%f,%f)", a.x, a.y, a.z, b.x, b.y, b.z);

    visualization_msgs::Marker points, line_strip;

    points.header.frame_id = line_strip.header.frame_id = "/laser";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "follow_path";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 1;
    line_strip.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.03;

    // Points are blue
    points.color.b = 1.0;
    points.color.a = 1.0;

    // Line strip is green
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;


    // why the hell can't i simply cast this points??
    geometry_msgs::Point pa, pb;
    pa.x = a.x; pa.y = a.y; pa.z = a.z;
    pb.x = b.x; pb.y = b.y; pb.z = b.z;

    points.points.push_back(pa);
    points.points.push_back(pb);
    line_strip.points.push_back(pa);
    line_strip.points.push_back(pb);

    publish_rviz_marker_.publish(points);
    publish_rviz_marker_.publish(line_strip);
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
