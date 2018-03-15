#ifndef ELEVATION_MAP_H
#define ELEVATION_MAP_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <ros/time.h>
#include <sensor_msgs/Image.h>


namespace tf
{
class Transform;
}

namespace cv
{
class Mat;

}

/**
 * @brief The ObstacleCloud class represents all currently known obstacles.
 */
class ElevationMap
{
public:
    using Ptr = std::shared_ptr<ElevationMap>;
    using ConstPtr = std::shared_ptr<ElevationMap const>;

    // PointXY would be sufficient, but buildin transform of point clouds works only with XYZ-points...
    using EMap = sensor_msgs::ImageConstPtr;
    using EMapType = sensor_msgs::Image;

    /**
     * @brief cloud the current obstacle cloud
     */
    EMap elevationMap;

    ElevationMap();
    ElevationMap(const EMap & c);

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
     * @brief toCVMat
     * @return the opencv mat
     */
    cv::Mat toCVMat() const;


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

#endif // ELEVATION_MAP_H
