#ifndef OBSTACLECLOUD_HPP
#define OBSTACLECLOUD_HPP

#include <pcl_ros/point_cloud.h>

// PointXY would be sufficient, but buildin transform of point clouds works only with XYZ-points...
typedef pcl::PointXYZ ObstaclePoint;
typedef pcl::PointCloud<ObstaclePoint> ObstacleCloud;

#endif // OBSTACLECLOUD_HPP
