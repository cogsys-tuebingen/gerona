#ifndef UTILS_DEM2PC_H
#define UTILS_DEM2PC_H


#include "sensor_msgs/PointCloud.h"
#include "pcl_ros/point_cloud.h"

/**
 * @brief Helper class for DEM to point cloud conversion
 */
class UtilsDem2PC
{

public:

static float ClampIntensity(float val)
{
    float cloudOutputMinval_ = -0.5;
    float cloudOutputMaxval_ = 0.5;
    float cloudOutputRange_ = cloudOutputMaxval_-cloudOutputMinval_;

    if (val < cloudOutputMinval_) return 0;
    if (val > cloudOutputMaxval_) return 255;
    return ((val-cloudOutputMinval_) / (cloudOutputRange_))*255.0f;


}

static pcl::PointCloud<pcl::PointXYZRGB> ImageToCloud(const cv::Mat &inImage, const cv::Point2f &origin, const float pixelResolution )
{
    pcl::PointCloud<pcl::PointXYZRGB> res;
    res.width = inImage.cols;
    res.height = inImage.rows;
    res.resize(res.width*res.height);
    //res.
    const float* imgPtr;

    float curPosY = origin.y,curPosX = origin.x;


    for (int y = 0; y <  inImage.rows;y++)
    {
        curPosX = origin.x;
        imgPtr = inImage.ptr<float>(y);
        for (int x = 0; x <  inImage.cols;x++)
        {
            float val = ((*imgPtr)-10000.0f)/1000.0f;
            pcl::PointXYZRGB tp;
            tp.x = curPosX;
            tp.y = curPosY;
            tp.z = val;

            tp.r = ClampIntensity(val);
            tp.g = ClampIntensity(val);
            tp.b = ClampIntensity(val);

            res.at(x,y) = tp;

            curPosX+= 1.0/pixelResolution;
            imgPtr++;

        }
        curPosY+= 1.0/pixelResolution;
    }

    return res;



}


static void PublishCloud(ros::Time stamp, std::string demFrame, cv::Mat &dem, ros::Publisher &imageCloud_pub_, const cv::Point2f &origin, const float pixelResolution)
{
    pcl::PointCloud<pcl::PointXYZRGB> demCloud = ImageToCloud(dem, origin, pixelResolution);

    sensor_msgs::PointCloud2 demCloudMsg;
    pcl::toROSMsg(demCloud,demCloudMsg);
    demCloudMsg.header.stamp = stamp;
    demCloudMsg.header.frame_id = demFrame;
    imageCloud_pub_.publish(demCloudMsg);
}


};

#endif
