#include <ros/ros.h>
#include <path_follower/pathfollower.h>
#include <path_follower/path_follower_server.h>
#include <path_follower/utils/parameters.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    PathFollower pf(nh);
    PathFollowerServer server(pf);

    // print table of parameters to /tmp/parameters.md
    Parameters::printToFileAllInstances();

    server.spin();

    return 0;
}


