#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include "calibrationdatastorage.h"
#include "scanfeaturecalculator.h"

using namespace std;

ScanFeatureCalculator sfc;
ros::Publisher pub[4];


void callback(const sensor_msgs::LaserScanConstPtr &scan, int layer)
{
    sfc.setScan(*scan, layer);
    pub[layer].publish(sfc.getPreprocessedScan());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "training_data_extractor");
    ros::NodeHandle nh;

    // use default calibration file - no parameter here, add only if needed.
    string calib_file = ros::package::getPath(ROS_PACKAGE_NAME) + std::string("/rangecalibration.yaml");
    ROS_INFO("Using calibration file %s", calib_file.c_str());

    // look for existing range calibration file
    CalibrationDataStorage cds(calib_file);
    vector<vector<float> > calib_data;
    cds.load(&calib_data);

    sfc.setCalibrationScan(calib_data);

    ros::Subscriber s0 = nh.subscribe<sensor_msgs::LaserScan>("/sick_ldmrs/scan0", 0, boost::bind(&callback, _1, 0));
    ros::Subscriber s1 = nh.subscribe<sensor_msgs::LaserScan>("/sick_ldmrs/scan1", 0, boost::bind(&callback, _1, 1));
    ros::Subscriber s2 = nh.subscribe<sensor_msgs::LaserScan>("/sick_ldmrs/scan2", 0, boost::bind(&callback, _1, 2));
    ros::Subscriber s3 = nh.subscribe<sensor_msgs::LaserScan>("/sick_ldmrs/scan3", 0, boost::bind(&callback, _1, 3));

    pub[0] = nh.advertise<sensor_msgs::LaserScan>("/scan0", 0);
    pub[1] = nh.advertise<sensor_msgs::LaserScan>("/scan1", 0);
    pub[2] = nh.advertise<sensor_msgs::LaserScan>("/scan2", 0);
    pub[3] = nh.advertise<sensor_msgs::LaserScan>("/scan3", 0);

    ros::spin();

    return 0;
}

