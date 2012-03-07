#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class DisplayLaserData
{
public:
    DisplayLaserData();

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;

    void printLaserData(const sensor_msgs::LaserScan::Ptr &msg);
    void calibrate();
};

#endif // DISPLAY_LASER_DATA_H
