#ifndef TRAINING_DATA_EXTRACTOR_H
#define TRAINING_DATA_EXTRACTOR_H

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "scanfeaturecalculator.h"

class TrainingDataExtractor
{
public:
    TrainingDataExtractor();

    void generateSamples();

private:
    struct Interval
    {
        float from, to;
    };

    struct Sample
    {
        std::vector<float> range_variance;
        std::vector<float> range_derivative;
        std::vector<float> intensity_derivative;
    };

    //! Default path/name of the range calibration file
    const static std::string DEFAULT_RANGE_CALIBRATION_FILE;
    int sample_size_;

    //! List of all samples
    std::vector<Sample> samples_;

    //! Calculates scan features
    ScanFeatureCalculator feature_calculator_;


    void processScan(const sensor_msgs::LaserScanConstPtr &scan, int layer);
};

#endif // TRAINING_DATA_EXTRACTOR_H
