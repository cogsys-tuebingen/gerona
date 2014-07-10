#include "scan2cloud.h"

ScanConverter::ScanConverter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 10, &ScanConverter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
}

void ScanConverter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    if(!tfListener_.waitForTransform(
          scan_in->header.frame_id,
          "/base_link",
          scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
          ros::Duration(1.0))){
       return;
    }
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}
