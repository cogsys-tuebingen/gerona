#include <ros/ros.h>
#include <path_follower/pathfollower.h>
#include <path_follower/utils/parameters.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh;

    PathFollower pf(nh);

//    Parameters::printToFileAllInstances();

    pf.spin();

    return 0;
}


