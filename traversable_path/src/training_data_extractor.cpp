#include "training_data_extractor.h"
#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdlib>
#include <ctime>
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
    ros::param::param<int>("~sample_size", sample_size_, 20);

    loadClassifications();

    // load range calibration filename from parameter
    string calib_file;
    ros::param::param<string>("~calibration_file", calib_file, DEFAULT_RANGE_CALIBRATION_FILE);
    if (calib_file.compare("default") == 0) {
        calib_file = DEFAULT_RANGE_CALIBRATION_FILE;
    }
    ROS_INFO("Using calibration file %s", calib_file.c_str());

    // look for existing range calibration file
    CalibrationDataStorage cds(calib_file);
    vector<vector<float> > calib_data;
    cds.load(&calib_data);
    feature_calculator_.setCalibrationScan(calib_data);


    // set seed for random generator
    srand(time(NULL));
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
            // extract layer number from topic name (last char)
            string topic = m.getTopic();
            int layer = topic[topic.length()-1] - '0';

            ROS_INFO("Process scan of layer %d. min angle: %g", layer, scan->angle_min);

            processScan(scan, layer);
        }
    }

    bag.close();


    // finally write the samples to an ARFF file
    writeArffFile();
}

void TrainingDataExtractor::loadClassifications()
{
    string classification_file;
    ros::param::get("~classification_file", classification_file);

    ifstream file(classification_file.data());

    if (!file.good()) {
        throw runtime_error("Could not open classification file");
    }

    // Start and end of 'border intervals'
    string slayer, istart, iend;
    while (getline(file, slayer, ',')) {
        getline(file, istart, ',');
        getline(file, iend);
        int layer = boost::lexical_cast<int>(slayer);
        float fistart = boost::lexical_cast<float>(istart);
        float fiend   = boost::lexical_cast<float>(iend);

        // do not rely on correct order of istart and iend -> take min and max
        pair<float, float> interval;
        interval.first  = min(fistart, fiend);
        interval.second = max(fistart, fiend);
        borders_[layer].push_back(interval);

        ROS_INFO("Border in interval [%g, %g]", interval.first, interval.second);
    }

    file.close();
}

bool TrainingDataExtractor::getClassification(float angle, int layer)
{
    vector<pair<float,float> >::const_iterator iter;
    for(iter = borders_[layer].begin(); iter != borders_[layer].end(); ++iter) {
        if ( (iter->first <= angle) && (angle <= iter->second) ) {
            return PATH_BORDER;
        }
    }
    return NO_PATH_BORDER;
}

void TrainingDataExtractor::processScan(const sensor_msgs::LaserScanConstPtr &scan, int layer)
{
    feature_calculator_.setScan(*scan, layer);

    // calculate features
    vector<float> range_deriv = feature_calculator_.rangeDerivative();
    vector<float> range_variance = feature_calculator_.rangeVariance();
    vector<float> intensity_deriv = feature_calculator_.intensityDerivative();

    // iterate not over full scan but start with a half-window-size offset
    size_t n = range_deriv.size();
    size_t range_start = sample_size_ / 2 + 1;
    size_t range_end   = n - range_start;

    std::vector<Sample> pos_samples;
    std::vector<Sample> neg_samples;

    for (size_t i = range_start; i < range_end; ++i) {
        // generate the sample for point i (it contains all points in the window around i and the classification of i)

        Sample s;
        //TODO: use assign() to avoid this loop
        for (size_t j = 0; j < (size_t)sample_size_; ++j) {
            size_t ind = i - sample_size_/2 + j;

            // multiply everything with 1000, because weka can't discretize numbers that are equal to the 6th past-comma-digit
            s.range_variance.push_back(range_variance[ind] * 1000);
            s.range_derivative.push_back(range_deriv[ind] * 1000);
            s.intensity_derivative.push_back(intensity_deriv[ind] * 1000);
        }

        float angle = scan->angle_min + scan->angle_increment * i;
        s.classification = getClassification(angle, layer);

        if (s.classification)
            pos_samples.push_back(s);
        else
            neg_samples.push_back(s);
    }

    // balance number of positive and negative samples
    balanceSamples(pos_samples, neg_samples);

    // add to global sample list
    samples_.insert(samples_.end(), pos_samples.begin(), pos_samples.end());
    samples_.insert(samples_.end(), neg_samples.begin(), neg_samples.end());
}

void TrainingDataExtractor::balanceSamples(const std::vector<Sample> &pos_samples, std::vector<Sample> &neg_samples)
{
    while (neg_samples.size() > pos_samples.size()) {
        size_t i = rand() % neg_samples.size();
        neg_samples.erase(neg_samples.begin() + i);
    }
}

void TrainingDataExtractor::writeArffFile()
{
    string filename;
    ros::param::param<string>("~output_file", filename, "tp_training_data.arff");
    ofstream file(filename.data(), ios::trunc);

    if (!file.good()) {
        throw runtime_error("Can't open output fiel for writing :(");
    }

    // commentary header
    std::string bagfile; ros::param::get("~bagfile", bagfile);
    file << "% traversable_path - Training data for border detection\n"
            "% Generated automaticly from " << bagfile << endl;

    //TODO: anpassen fuer border_[layer]
//    file << "% Border intervals used for classification: ";
//    vector<pair<float,float> >::const_iterator iter;
//    for(iter = borders_.begin(); iter != borders_.end(); ++iter) {
//        file << "[" << iter->first<< ", " << iter->second << "] ";
//    }
    file << endl;


    // declarations
    file << "@relation tp_path_borders_" << bagfile << endl << endl;

    for (int i = 0; i < sample_size_; ++i) {
        file << "@attribute range_var" << i << " numeric" << endl;
        file << "@attribute range_deriv" << i << " numeric" << endl;
        file << "@attribute intensity_deriv" << i << " numeric" << endl;
    }
    file << "@attribute class {path_border, no_path_border}" << endl;

    // data
    file << "\n@data" << endl;
    vector<Sample>::const_iterator sample_it;
    for (sample_it = samples_.begin(); sample_it != samples_.end(); ++sample_it) {
        for (int i = 0; i < sample_size_; ++i) {
            file << sample_it->range_variance[i] << ","
                 << sample_it->range_derivative[i] << ","
                 << sample_it->intensity_derivative[i] << ",";
        }
        file << (sample_it->classification == PATH_BORDER ? "path_border" : "no_path_border") << endl;
    }
}
