/// HEADER
#include <path_follower/utils/obstacle_cloud.h>

#include <pcl_ros/point_cloud.h>

ObstacleCloud::ObstacleCloud()
    : cloud(new Cloud)
{}

ObstacleCloud::ObstacleCloud(const Cloud::Ptr& c)
    : cloud(c)
{
}
ObstacleCloud::ObstacleCloud(const Cloud::ConstPtr& c)
    : cloud(new Cloud(*c))
{
}

bool ObstacleCloud::empty() const
{
    return cloud->empty();
}

void ObstacleCloud::clear()
{
    return cloud->clear();
}
