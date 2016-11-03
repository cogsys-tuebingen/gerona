#include <ros/ros.h>
#include <path_follower/pathfollower.h>
#include <path_follower/path_follower_server.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <pcl_ros/point_cloud.h>

namespace {
void importCloud(const ObstacleCloud::Cloud::ConstPtr& cloud, PathFollower* pf)
{
    ObstacleCloud::Ptr ocloud = std::make_shared<ObstacleCloud>(cloud);
    pf->obstacleCloudCB(ocloud);
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;


    PathFollower pf(nh);
    PathFollowerServer server(pf);

    ros::Subscriber obstacle_cloud_sub_ =
            nh.subscribe<ObstacleCloud::Cloud>("/obstacles", 10,
                                        boost::bind(&importCloud, _1, &pf));


    // print table of parameters to /tmp/parameters.md
    Parameters::printToFileAllInstances();

    server.spin();

    return 0;
}


