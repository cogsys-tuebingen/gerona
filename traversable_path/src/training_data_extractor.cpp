#include "training_data_extractor.h"
#include <string>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include "calibrationdatastorage.h"

using namespace std;


const std::string TrainingDataExtractor::DEFAULT_RANGE_CALIBRATION_FILE = ros::package::getPath(ROS_PACKAGE_NAME)
                                                             + std::string("/rangecalibration.yaml");

TrainingDataExtractor::TrainingDataExtractor()
{
//    string classification_file;
//    ros::param::get("~classification_file", classification_file);

    ros::param::param<int>("~sample_size", sample_size_, 20);


    // load range calibration filename from parameter
    string calib_file;
    ros::param::param<string>("calibration_file", calib_file, DEFAULT_RANGE_CALIBRATION_FILE);
    if (calib_file.compare("default") == 0) {
        calib_file = DEFAULT_RANGE_CALIBRATION_FILE;
    }
    ROS_INFO("Using calibration file %s", calib_file.c_str());

    // look for existing range calibration file
    CalibrationDataStorage cds(calib_file);
    vector<vector<float> > calib_data;
    cds.load(&calib_data);
    feature_calculator_.setCalibrationScan(calib_data);
}

void TrainingDataExtractor::generateSamples()
{
    // get name of bagfile from parameter
    std::string bagfile;
    ros::param::get("~bagfile", bagfile);

    // open bagfile
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    // specifiy topics
    std::vector<std::string> topics;
    topics.push_back(std::string("/sick_ldmrs/scan0"));
    topics.push_back(std::string("/sick_ldmrs/scan1"));
    topics.push_back(std::string("/sick_ldmrs/scan2"));
    topics.push_back(std::string("/sick_ldmrs/scan3"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));


    // iterate over messages
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
        if (scan != NULL) {
            string topic = m.getTopic();
            int layer = topic[topic.length()-1] - '0';

            processScan(scan, layer);
        }
    }


    bag.close();
}

void TrainingDataExtractor::processScan(const sensor_msgs::LaserScanConstPtr &scan, int layer)
{
    feature_calculator_.setScan(*scan, layer);

    vector<float> range_deriv = feature_calculator_.rangeDerivative();
    vector<float> range_variance = feature_calculator_.rangeVariance();
    vector<float> intensity_deriv = feature_calculator_.intensityDerivative();

    int n = range_deriv.size();
    int range_start = sample_size_ / 2 + 1;
    int range_end   = n - range_start;

    for (size_t i = range_start; i < range_end; ++i) {
        Sample s;
        //TODO: use assign() to avoid this loop
        for (size_t j = 0; j < sample_size_; ++j) {
            size_t ind = i - sample_size_/2 + j;

            s.range_variance.push_back(range_variance[ind]);
            s.range_derivative.push_back(range_deriv[ind]);
            s.intensity_derivative.push_back(intensity_deriv[ind]);
        }
        samples_.push_back(s);
    }

    ////// TODO: Hier gehts weiter
}
