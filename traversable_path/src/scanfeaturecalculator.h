#ifndef SCANFEATURECALCULATOR_H
#define SCANFEATURECALCULATOR_H

#include <vector>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/LaserScan.h>

struct PointFeatures
{
    float range_derivative_;
    float intensity_derivative_;
    float range_variance_;
};

class ScanFeatureCalculator
{
public:
    typedef sensor_msgs::LaserScan LaserScan;
    typedef std::vector<float> ScanData;

    ScanFeatureCalculator():
        variance_window_size_(20)
    {}

    void setCalibrationScan(std::vector<ScanData> plane_ranges)
    {
        plane_ranges_ = plane_ranges;
    }

    void setVarianceWindowSize(unsigned int window_size)
    {
        variance_window_size_ = window_size;
    }

    void setScan(LaserScan scan, int layer);

    LaserScan getPreprocessedScan()
    {
        return scan_;
    }


    const ScanData& rangeDerivative();
    const ScanData& intensityDerivative();

    //! Calculate for each element in data the variance of the window around the element.
    const ScanData& rangeVariance();

private:
    //! Range data of a (preferably) perfect plane, to calibrate the laser data.
    std::vector<ScanData> plane_ranges_;

    unsigned int variance_window_size_;

    LaserScan scan_;
    ScanData range_derivative_;
    ScanData intensity_derivative_;
    ScanData range_variance_;


    void preprocess(int layer);

    /**
     * @brief Calculates average of the elements of a float list.
     * @param A list of float-values.
     * @return Average of the list-values.
     */
    static float avg(const boost::circular_buffer<float> &xs);

    /**
     * @brief Smoothes the curve describted by data.
     */
    static std::vector<float> smooth(std::vector<float> data, const unsigned int num_values);

    void calcDerivative();

    static float variance(const boost::circular_buffer<float> &window);
};

#endif // SCANFEATURECALCULATOR_H
