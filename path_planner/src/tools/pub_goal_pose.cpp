/// COMPONENT
#include "../planner_node.h"

/// SYSTEM
#include <math.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

namespace {
geometry_msgs::PoseStamped tf2pose(const tf::Transform& pose)
{
    geometry_msgs::PoseStamped r;
    tf::poseTFToMsg(pose, r.pose);
    return r;
}
}


bool getWorldPose( tf::TransformListener& pose_listener, const std::string& world_frame, const std::string& robot_frame, geometry_msgs::PoseStamped& pose_msg)
{
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg;
    ros::spinOnce();
    try {
        pose_listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform robot pose: %s", ex.what());
        return false;
    }
    tf::transformStampedTFToMsg(transform, msg);


    pose_msg.pose.position.x = msg.transform.translation.x;
    pose_msg.pose.position.y = msg.transform.translation.y;
    pose_msg.pose.position.z = msg.transform.translation.z;
    pose_msg.pose.orientation = msg.transform.rotation;
    pose_msg.header.frame_id=world_frame;
    return true;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");
    std::string goal_topic("/move_base_simple/goal");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic,10);

    geometry_msgs::PoseStamped pose;
    tf::TransformListener pose_listener;



    ros::WallRate r(2);
    int counter =0;
    while(ros::ok()){
        ros::spinOnce();
        bool status=getWorldPose(pose_listener,"map","base_link",pose);
        if (status) {
            ROS_INFO("publish pose\n");
            goal_pub.publish(pose);
            counter++;
        }
        r.sleep();

        if (counter>=1) break;
    }
}
