#include <ros/ros.h>
#include "pathfollower.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh;

    MotionControlNode pf(nh);

    ros::Rate rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        pf.update();
        rate.sleep();
    }

    return 0;
}


