#ifndef OBSTACLE_CLOUD_H
#define OBSTACLE_CLOUD_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

namespace pcl
{
struct PointXYZ;

template <typename T>
class PointCloud;
}

class ObstacleCloud
{
public:
    using Ptr = std::shared_ptr<ObstacleCloud>;
    using ConstPtr = std::shared_ptr<ObstacleCloud const>;

    // PointXY would be sufficient, but buildin transform of point clouds works only with XYZ-points...
    using ObstaclePoint = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<ObstaclePoint>;

    boost::shared_ptr<Cloud> cloud;

    ObstacleCloud();
    ObstacleCloud(const boost::shared_ptr<Cloud>& c);
    ObstacleCloud(const boost::shared_ptr<Cloud const>& c);

    bool empty() const;
    void clear();
};

#endif // OBSTACLE_CLOUD_H
