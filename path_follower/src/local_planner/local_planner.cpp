/// HEADER
#include <path_follower/local_planner/local_planner.h>

LocalPlanner::LocalPlanner(PathFollower &follower, tf::Transformer &transformer)
    : follower_(follower), transformer_(transformer)
{

}

LocalPlanner::~LocalPlanner()
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

void LocalPlanner::setObstacleCloud(const ObstacleCloud::ConstPtr &msg)
{
    obstacle_cloud_ = msg;

    ros::Time now = ros::Time::now();

    if(!transformer_.waitForTransform("odom", "base_link", now, ros::Duration(0.005))) {
        ROS_WARN_THROTTLE_NAMED(1, "global_path", "cannot transform base_link to odom");
        return;
    }
    transformer_.lookupTransform("odom", "base_link", now, base_to_odom);
    odom_to_base = base_to_odom.inverse();
}
