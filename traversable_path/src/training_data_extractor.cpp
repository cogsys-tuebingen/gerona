#include "training_data_extractor.h"
#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdlib>
#include <ctime>
#include <numeric>
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
    ros::param::param<int>("~sample_size", sample_size_, 15);

    feature_calculator_ = ScanFeatureCalculator(sample_size_);

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

        float fistart, fiend;

        if (istart.compare("inf") == 0) {
            fistart = INFINITY;
        } else if (istart.compare("-inf") == 0) {
            fistart = -INFINITY;
        } else {
            fistart = boost::lexical_cast<float>(istart);
        }

        if (iend.compare("inf") == 0) {
            fiend = INFINITY;
        } else if (iend.compare("-inf") == 0) {
            fiend = -INFINITY;
        } else {
            fiend = boost::lexical_cast<float>(iend);
        }

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
            return UNTRAVERSABLE;
        }
    }
    return TRAVERSABLE;
}

void TrainingDataExtractor::processScan(const sensor_msgs::LaserScanConstPtr &scan, int layer)
{
    feature_calculator_.setScan(*scan, layer);

    vector<PointFeatures> samples = feature_calculator_.getPointFeatures();
    std::vector<Sample> pos_samples;
    std::vector<Sample> neg_samples;
    for (vector<PointFeatures>::iterator sample_it = samples.begin(); sample_it != samples.end(); ++sample_it) {
        Sample s;
        s.features = *sample_it;
        s.classification = getClassification(sample_it->angle, layer);

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
    bool print_header;
    ros::param::param<bool>("~print_header", print_header, true);

    string filename;
    ros::param::param<string>("~output_file", filename, "tp_training_data.arff");
    ofstream file(filename.data(), ios::trunc);

    if (!file.good()) {
        throw runtime_error("Can't open output fiel for writing :(");
    }


    // commentary header
    std::string bagfile; ros::param::get("~bagfile", bagfile);
    file << "% traversable_path - Training data for border detection\n"
            "% training_data_extractor version 2\n"
            "% Generated automaticly from " << bagfile << endl;

    //TODO: anpassen fuer border_[layer]
//    file << "% Border intervals used for classification: ";
//    vector<pair<float,float> >::const_iterator iter;
//    for(iter = borders_.begin(); iter != borders_.end(); ++iter) {
//        file << "[" << iter->first<< ", " << iter->second << "] ";
//    }
    file << endl;


    if (print_header) {
        // declarations
        file << "@relation tp_path_borders_" << bagfile << endl << endl;

        file << "@attribute angle numeric" << endl;
        for (int i = 0; i < sample_size_; ++i) {
            file << "@attribute range_var" << i << " numeric" << endl;
            file << "@attribute range_deriv" << i << " numeric" << endl;
            file << "@attribute intensity_deriv" << i << " numeric" << endl;
        }
        file << "@attribute class {path_border, no_path_border}" << endl;

        // data
        file << "\n@data" << endl;
    }

    // data
    vector<Sample>::const_iterator sample_it;
    for (sample_it = samples_.begin(); sample_it != samples_.end(); ++sample_it) {
        file << sample_it->features.angle << ",";
        for (int i = 0; i < sample_size_; ++i) {
            file << sample_it->features.range_variance[i] << ","
                 << sample_it->features.range_derivative[i] << ","
                 << sample_it->features.intensity_derivative[i] << ",";
        }
        file << (sample_it->classification == UNTRAVERSABLE ? "path_border" : "no_path_border") << endl;
    }
}
