/// HEADER
#include <path_follower/utils/obstacle_cloud.h>

#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

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

void ObstacleCloud::transformCloud(const tf::Transform& transform)
{
    for(auto& pt : cloud->points) {
        tf::Point point(pt.x,pt.y,pt.z);
        tf::Point transformed = transform * point;
        pt.x = transformed.x();
        pt.y = transformed.y();
        pt.z = transformed.z();
    }
}
