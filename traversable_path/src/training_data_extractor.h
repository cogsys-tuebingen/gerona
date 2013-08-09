#ifndef TRAINING_DATA_EXTRACTOR_H
#define TRAINING_DATA_EXTRACTOR_H

#include <vector>
#include <utility>
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
        bool classification;
    };

    const static bool PATH_BORDER = true;
    const static bool NO_PATH_BORDER = false;


    //! Default path/name of the range calibration file
    const static std::string DEFAULT_RANGE_CALIBRATION_FILE;
    int sample_size_;

    //! List of angular intervals that contain borders
    std::vector<std::pair<float,float> > borders_[4];

    //! List of all samples.
    std::vector<Sample> samples_;

    //! Calculates scan features
    ScanFeatureCalculator feature_calculator_;

    /**
     * @brief Load classification from simple csv file
     *
     * The classification is given as a cvs-file containing a set of angular intervals which cover the borders in the
     * scan data. The file has to look like this:
     *    layer,int1_start,int1_end
     *    layer,int2_start,int2_end
     *    ...
     *
     * for example:
     *    0,-2.56,-2.4
     *    0,0.2,0.4
     *    1,1.5,1.69
     */
    void loadClassifications();

    /**
     * @brief Returns the classification of a point at the specified angle. loadClassification() has to be called before!
     * @param angle
     * @return classification (PATH_BORDER or NO_PATH_BORDER)
     */
    bool getClassification(float angle, int layer);

    //! Creates the samples from the scan.
    void processScan(const sensor_msgs::LaserScanConstPtr &scan, int layer);

    /**
     * @brief Drop samples until the number positive and negative samples are the same.
     *
     * Since the path borders cover only a very small part of a full laserscan there will be much more negative
     * (no border) samples than positive ones. This is bad for learning, so this method drops randomly chosen negative
     * samples until the numbers are equal.
     */
    void balanceSamples(const std::vector<Sample> &pos_samples, std::vector<Sample> &neg_samples);

    //! Write samples to ARFF file. (processScan() has to be called before!)
    void writeArffFile();
};

#endif // TRAINING_DATA_EXTRACTOR_H
