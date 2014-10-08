#include <ros/ros.h>
#include "pathcontroller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_control_node");
    ros::NodeHandle nh;

    PathController pc(nh);

    ros::Rate r(60);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

