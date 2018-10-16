
#ifndef MULTI_SCAN2CLOUD_H
#define MULTI_SCAN2CLOUD_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include <sensor_msgs/LaserScan.h>


class ScanProcessor
{
public:
    ScanProcessor();

    void SetParams(const ros::NodeHandle &nh);

    void ProcessScan(const sensor_msgs::LaserScan &scan, const std::vector<bool> scanMask, const tf::Point angleMinMax, std::vector<tf::Point> &out_points);
    void TransformCloud(const tf::Transform& transform, const std::vector<tf::Point> &in, std::vector<tf::Point> &out);

    void TransformCloud(const std::vector<tf::Point> &in, const std::string frame_id, const ros::Time stamp, std::vector<tf::Point> & out);
    void CreateCloud(const std::vector<tf::Point> &obstacle_points, const std::string frame_id, const ros::Time stamp, sensor_msgs::PointCloud2 &cloud);
    void CreateCloud(const std::vector<tf::Point> &obstacle_points1, const std::vector<tf::Point> &obstacle_points2, const std::string frame_id, const ros::Time stamp, sensor_msgs::PointCloud2 &cloud);
    void ToPoints(const sensor_msgs::LaserScan &scan, const std::vector<bool> &scan_mask, std::vector<tf::Point> &points);
    void ToPoints(const sensor_msgs::LaserScan &scan, std::vector<tf::Point> &points);
    void ToPoints(const sensor_msgs::LaserScan &scan, const float angleMin, const float angleMax, std::vector<tf::Point> &points);

    float tukey_k_;
    float threshold_w_;
    int windowSize_;
    bool use_dist_;
    int filterType_;
    int minPoints_;
    float minSegmentSize_;
    bool always_use_latest_transform_obstacles_;
    bool useMask_;
    std::string fixed_frame_;
    float tf_timeout_;
    float minRange_;
    //float angleFilterMin_,angleFilterMax_;
    bool useAngleFilter_;
private:
    std::vector<tf::Point> points1_;
    std::vector<tf::Point> points2_;
    tf::TransformListener tfListener_;


};


#endif
