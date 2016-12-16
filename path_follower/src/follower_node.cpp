#include <ros/ros.h>
#include <path_follower/pathfollower.h>
#include <path_follower/path_follower_server.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/utils/pose_tracker.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

namespace {
void importCloud(const ObstacleCloud::Cloud::ConstPtr& sensor_cloud, PathFollower* pf)
{
    ros::Time now;
    now.fromNSec(sensor_cloud->header.stamp * 1e3);

    std::string sensor_frame = sensor_cloud->header.frame_id;
    if(sensor_frame.at(0) == '/') {
        sensor_frame = sensor_frame.substr(1);
    }

    auto& pose_tracker = pf->getPoseTracker();
    tf::Transform fixed_to_sensor = pose_tracker.getRelativeTransform(pose_tracker.getFixedFrameId(), sensor_frame, now, ros::Duration(0.1));

    auto obstacle_cloud = std::make_shared<ObstacleCloud>(sensor_cloud);
    obstacle_cloud->transformCloud(fixed_to_sensor);
    pf->setObstacles(obstacle_cloud);
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


