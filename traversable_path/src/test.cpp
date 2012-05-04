/**
 * @brief Only for testing some things.
 */
#include <stdio.h>
#include <ros/ros.h>
#include "pointclassification.h"
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node_handle;

    //tf::Quaternion q;
    geometry_msgs::PoseStampedConstPtr q;
    while(ros::ok()) {
        q = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/traversable_path/goal", node_handle);
        cout << q->pose.orientation.x << ", " << q->pose.orientation.y << ", " << q->pose.orientation.z << ", " << q->pose.orientation.w << endl;
    }
    //cout << tf::getYaw(q) << endl;

    return 0;
}

