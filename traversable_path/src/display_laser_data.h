#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

class DisplayLaserData
{
public:
    DisplayLaserData();

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::ServiceServer service;
    bool isCalibrated;
    std::vector<float> planeRanges;

    void printLaserData(const sensor_msgs::LaserScanPtr &msg);
    bool calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

#endif // DISPLAY_LASER_DATA_H
