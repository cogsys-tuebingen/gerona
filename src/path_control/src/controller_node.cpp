#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_control_node");
    ros::NodeHandle nh;

    // do something here

    ros::spin();
    return 0;
}

