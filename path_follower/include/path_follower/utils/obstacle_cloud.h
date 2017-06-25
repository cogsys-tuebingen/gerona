#ifndef OBSTACLE_CLOUD_H
#define OBSTACLE_CLOUD_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <ros/time.h>

namespace tf
{
class Transform;
}

namespace pcl
{
struct PointXYZ;

template <typename T>
class PointCloud;
}

/**
 * @brief The ObstacleCloud class represents all currently known obstacles.
 */
class ObstacleCloud
{
public:
    using Ptr = std::shared_ptr<ObstacleCloud>;
    using ConstPtr = std::shared_ptr<ObstacleCloud const>;

    // PointXY would be sufficient, but buildin transform of point clouds works only with XYZ-points...
    using ObstaclePoint = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<ObstaclePoint>;

    /**
     * @brief cloud the current obstacle cloud
     */
    boost::shared_ptr<Cloud> cloud;

    ObstacleCloud();
    ObstacleCloud(const boost::shared_ptr<Cloud>& c);
    ObstacleCloud(const boost::shared_ptr<Cloud const>& c);

    /**
     * @brief empty
     * @return true, iff no obstacle exists
     */
    bool empty() const;

    /**
     * @brief clear removes all current obstacles.
     */
    void clear();

    /**
     * @brief transformCloud applies <transform> to all points in the current cloud
     * @param transform a transformation to apply to all points
     * @param target_frame, the new frame to set for this cloud
     */
    void transformCloud(const tf::Transform& transform, const std::string& target_frame);

    /**
     * @brief getStamp
     * @return the time stamp of this cloud
     */
    ros::Time getStamp() const;

    /**
     * @brief getFrameId
     * @return the frame id of this cloud
     */
    std::string getFrameId() const;
};

#endif // OBSTACLE_CLOUD_H
