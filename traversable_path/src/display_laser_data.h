#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include <string>
#include <list>
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

class DisplayLaserData
{
public:
    DisplayLaserData();

private:
    const static std::string RANGE_CALIBRATION_FILE;

    ros::NodeHandle node_handle_;
    ros::Publisher pub_;
    ros::Publisher publish_smooth_;
    ros::Publisher publish_differential_;
    ros::Subscriber sub_;
    ros::ServiceServer service_;
    bool is_calibrated_;
    std::vector<float> plane_ranges_;

    void printLaserData(const sensor_msgs::LaserScanPtr &msg);
    bool calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    std::vector<float> smooth(std::vector<float> data);
    float avg(std::list<float> &xs);
};

const std::string DisplayLaserData::RANGE_CALIBRATION_FILE = ros::package::getPath(ROS_PACKAGE_NAME)
                                                             + std::string("/rangecalibration.dat");

#endif // DISPLAY_LASER_DATA_H
