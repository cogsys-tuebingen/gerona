#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include <string>
#include <vector>
#include <list>
#include <boost/circular_buffer.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include "pointclassification.h"
#include "visualization.h"

class DisplayLaserData
{
public:
    DisplayLaserData();

private:
    const static std::string DEFAULT_RANGE_CALIBRATION_FILE;

    ros::NodeHandle node_handle_;
    ros::Publisher publish_normalized_;
    ros::Publisher publish_smooth_;
    ros::Publisher publish_differential_;
    ros::Subscriber subscribe_laser_scan_;
    ros::ServiceServer service_;
    std::string range_calibration_file_;
    bool is_calibrated_;
    std::vector<float> plane_ranges_;
    Visualization visualizer_;

    void printLaserData(const sensor_msgs::LaserScanPtr &msg);
    bool calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /**
     * @brief Smoothes the curve describted by data.
     */
    std::vector<float> smooth(std::vector<float> data, const unsigned int num_values);

    /**
     * @brief Calculates average of the elements of a float list.
     * @param A list of float-values.
     * @return Average of the list-values.
     */
    float avg(boost::circular_buffer<float> &xs);

    std::vector<PointClassification> detectObstacles(sensor_msgs::LaserScan data, std::vector<float> &out);
};

const std::string DisplayLaserData::DEFAULT_RANGE_CALIBRATION_FILE = ros::package::getPath(ROS_PACKAGE_NAME)
                                                             + std::string("/rangecalibration.yaml");

#endif // DISPLAY_LASER_DATA_H
