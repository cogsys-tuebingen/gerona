#ifndef UTILS_DEM2PC_H
#define UTILS_DEM2PC_H


#include "sensor_msgs/PointCloud2.h"
#include <opencv2/core/core.hpp>


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




    static void CreateEmptyXYZCloud(int size, sensor_msgs::PointCloud2 &cloud)
    {
        cloud = sensor_msgs::PointCloud2();
        int pointSize = sizeof(float)*3;


        cloud.height = 1;
        cloud.width = size;
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        cloud.point_step = pointSize;
        cloud.row_step = cloud.point_step*cloud.width;

        sensor_msgs::PointField fieldx;
        fieldx.count = 1;
        fieldx.name =  'x';
        fieldx.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldx.offset = 0;
        sensor_msgs::PointField fieldy;
        fieldy.count = 1;
        fieldy.name =  'y';
        fieldy.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldy.offset = 4;
        sensor_msgs::PointField fieldz;
        fieldz.count = 1;
        fieldz.name =  'z';
        fieldz.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldz.offset = 8;

        cloud.fields.push_back(fieldx);
        cloud.fields.push_back(fieldy);
        cloud.fields.push_back(fieldz);

        cloud.data.resize(cloud.point_step*cloud.width);
    }


    static void ImageToCloud(const cv::Mat &inImage, const cv::Point2f &origin, const float pixelResolution, sensor_msgs::PointCloud2 &cloud)
    {


        //res.
        const float* imgPtr;
        float curPosY = origin.y,curPosX = origin.x;
        float val;
        int x,y;
        const float ovth = 1.0f/1000.0f;

        int numPoints  = 0;
        for (y = 0; y <  inImage.rows;y++)
        {
            curPosX = origin.x;
            imgPtr = inImage.ptr<float>(y);
            for (x = 0; x <  inImage.cols;x++)
            {
                val = ((*imgPtr)-10000.0f)*ovth;
                if (val > -9.0f) numPoints++;
                imgPtr++;
            }

        }

        CreateEmptyXYZCloud(numPoints,cloud);

        float* curP = (float*)&cloud.data.front();
        curPosY = origin.y;
        curPosX = origin.x;

        for (y = 0; y <  inImage.rows;y++)
        {
            curPosX = origin.x;
            imgPtr = inImage.ptr<float>(y);
            for (x = 0; x <  inImage.cols;x++)
            {
                val = ((*imgPtr)-10000.0f)*ovth;
                if (val > -9.0f)
                {
                    *curP = curPosX;
                    ++curP;
                    *curP = curPosY;
                    ++curP;
                    *curP = val;
                    ++curP;
                }

                curPosX+= 1.0/pixelResolution;
                imgPtr++;

            }
            curPosY+= 1.0/pixelResolution;
        }

    }

    static void CreateCloud(const std::vector<cv::Point3f> &obstacle_points, const std::string frame_id, const ros::Time stamp, ros::Publisher &imageCloud_pub_)
    {

        sensor_msgs::PointCloud2 cloud;
        CreateEmptyXYZCloud(obstacle_points.size(),cloud);
        cloud.header.frame_id = frame_id;
        cloud.header.stamp = stamp;

        float *dstPtr = (float*) &cloud.data.front();

        for(int t = 0; t < obstacle_points.size();++t) {
            const cv::Point3f &point = obstacle_points[t];
            *dstPtr = point.x;
            ++dstPtr;
            *dstPtr = point.y;
            ++dstPtr;
            *dstPtr = point.z;
            ++dstPtr;
        }

        imageCloud_pub_.publish(cloud);

    }


    static void PublishCloud(ros::Time stamp, std::string demFrame, cv::Mat &dem, ros::Publisher &imageCloud_pub_, const cv::Point2f &origin, const float pixelResolution)
    {
        sensor_msgs::PointCloud2 demCloudMsg;

        ImageToCloud(dem, origin, pixelResolution, demCloudMsg);

        demCloudMsg.header.stamp = stamp;
        demCloudMsg.header.frame_id = demFrame;
        imageCloud_pub_.publish(demCloudMsg);
    }


};

#endif
