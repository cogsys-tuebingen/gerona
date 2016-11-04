/// HEADER
#include <path_follower/local_planner/local_planner.h>
#include <pcl_ros/point_cloud.h>
#include <path_follower/utils/obstacle_cloud.h>

#include <path_follower/utils/pose_tracker.h>

LocalPlanner::LocalPlanner(RobotController& controller, PoseTracker& pose_tracker)
    : controller_(controller),
      pose_tracker_(pose_tracker),
      transformer_(pose_tracker_.getTransformListener())
{

}

LocalPlanner::~LocalPlanner()
{

}

void LocalPlanner::reset()
{

}

void LocalPlanner::setGlobalPath(Path::Ptr path)
{
    global_path_.interpolatePath(path,false);

    ros::Time now = ros::Time::now();

    if(transformer_.waitForTransform("map", "odom", now, ros::Duration(1.0))) {
        transformer_.lookupTransform("map", "odom", now, initial_map_to_odom_);
        return;
    }
    if(transformer_.waitForTransform("map", "odom", ros::Time(0), ros::Duration(1.0))) {
        ROS_WARN_NAMED("global_path", "cannot transform map to odom, using latest");
        transformer_.lookupTransform("map", "odom", ros::Time(0), initial_map_to_odom_);
        return;
    }

    ROS_ERROR_NAMED("global_path", "cannot transform map to odom");
}

bool LocalPlanner::isNull() const
{
    return false;
}

void LocalPlanner::setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg)
{
    last_obstacle_cloud_ = obstacle_cloud_;
    obstacle_cloud_ = msg;

    ros::Time now;
    now.fromNSec(msg->cloud->header.stamp * 1e3);

    lastbase_to_odom = base_to_odom;
    odom_to_lastbase = odom_to_base;
    if(!transformer_.waitForTransform("odom", "base_link", now, ros::Duration(0.1))) {
        ROS_WARN_THROTTLE_NAMED(1, "global_path", "cannot transform base_link to odom");
        return;
    }
    transformer_.lookupTransform("odom", "base_link", now, base_to_odom);
    odom_to_base = base_to_odom.inverse();
}
