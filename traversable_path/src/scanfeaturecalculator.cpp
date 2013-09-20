#include "scanfeaturecalculator.h"
#include <numeric>
#include "scancleaner.h"
#include <boost/foreach.hpp>

using namespace std;

void ScanFeatureCalculator::setScan(ScanFeatureCalculator::LaserScan scan, int layer)
{
    scan_ = scan;
    preprocess(layer);

    // clear buffered data
    range_derivative_.clear();
    intensity_derivative_.clear();
    range_variance_.clear();
}


const ScanFeatureCalculator::ScanData &ScanFeatureCalculator::rangeDerivative()
{
    if (range_derivative_.empty()) {
        calcDerivative();
    }
    return range_derivative_;
}

const ScanFeatureCalculator::ScanData &ScanFeatureCalculator::intensityDerivative()
{
    if (intensity_derivative_.empty()) {
        calcDerivative();
    }
    return intensity_derivative_;
}


const ScanFeatureCalculator::ScanData &ScanFeatureCalculator::rangeVariance()
{
    if (range_variance_.empty()) {
        boost::circular_buffer<float> window;

        ScanData::const_iterator data_it = scan_.ranges.begin() + (variance_window_size_/2);

        // init window
        ScanData::const_iterator begin = scan_.ranges.begin(); //!< This is necessary to get a const begin for assign()
        window.assign(variance_window_size_, begin, data_it);

        range_variance_.reserve(scan_.ranges.size());
        for (; data_it != scan_.ranges.end(); ++data_it) {
            window.push_back(*data_it);
            range_variance_.push_back( variance(window) );
        }

        while (window.size() > (variance_window_size_+1)/2) { // +1 to 'ceil' odd window_sizes (has no effect on even ones)
            window.pop_front();
            range_variance_.push_back( variance(window) );
        }

        assert(scan_.ranges.size() == range_variance_.size());
    }

    return range_variance_;
}

const std::vector<PointFeatures> ScanFeatureCalculator::getPointFeatures()
{
    // calculate all features
    rangeDerivative();
    rangeVariance();
    intensityDerivative();

    // iterate not over full scan but start with a half-window-size offset
    size_t n = range_derivative_.size();
    size_t range_start = feature_size_ / 2 + 1;
    size_t range_end   = n - range_start;

    std::vector<PointFeatures> samples;

    for (size_t i = range_start; i < range_end; ++i) {
        // generate the sample for point i (it contains all points in the window around i and the classification of i)

        PointFeatures s;
        //TODO: use assign() to avoid this loop
        for (size_t j = 0; j < (size_t)feature_size_; ++j) {
            size_t ind = i - feature_size_/2 + j;

            s.range_variance.push_back(range_variance_[ind]);
            s.range_derivative.push_back(range_derivative_[ind]);
            s.intensity_derivative.push_back(intensity_derivative_[ind]);
        }

        // standardize features
        standardizeData(s.range_variance);
        standardizeData(s.range_derivative);
        standardizeData(s.intensity_derivative);

        s.angle = scan_.angle_min + scan_.angle_increment * i;

        samples.push_back(s);
    }

    return samples;
}

void ScanFeatureCalculator::preprocess(int layer)
{
    /// clean, normalize and smooth the scan

    // first of all, remove invalid or obviosly wrong (too high) values
    ScanCleaner::clean(scan_);

    // subtract plane calibration values to normalize the range data
    for (size_t i=0; i < scan_.ranges.size(); ++i) {
        scan_.ranges[i] -= plane_ranges_[layer][i];

        // mirror on x-axis (to make it more intuitive)
        scan_.ranges[i] *= -1;
    }

    // smooth
    scan_.ranges = smooth(scan_.ranges, 6);
    scan_.intensities = smooth(scan_.intensities, 4);
}

float ScanFeatureCalculator::avg(const boost::circular_buffer<float> &xs)
{
    if (xs.empty())
        return 0.0;
    else
        return accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
}

vector<float> ScanFeatureCalculator::smooth(std::vector<float> data, const unsigned int num_values)
{
    //FIXME range check
    boost::circular_buffer<float> neighbourhood(2*num_values + 1);
    unsigned int length = data.size();

    // push first values to neighbourhood
    for (unsigned int i = 0; i < num_values && i < length; ++i) {
        neighbourhood.push_back(data[i]);
    }

    // push next
    for (unsigned int i = 0; i < length-num_values; ++i) {
        neighbourhood.push_back(data[i+num_values]);
        data[i] = avg(neighbourhood);
    }

    // nothing more to push
    for (unsigned int i = length-num_values; i < data.size(); ++i) {
        neighbourhood.pop_front();
        data[i] = avg(neighbourhood);
    }

    return data;
}

void ScanFeatureCalculator::calcDerivative()
{
    // constant values
    const size_t n = scan_.ranges.size(); //!< Length of the scan vector.

    range_derivative_.resize(n);
    intensity_derivative_.resize(n);

    // differentials
    for (unsigned int i=0; i < n - 1; ++i) {
        range_derivative_[i] = scan_.ranges[i] - scan_.ranges[i+1];
        intensity_derivative_[i] = abs(scan_.intensities[i] - scan_.intensities[i+1]); //< BEWARE of the abs()!
    }
    range_derivative_[n-1] = scan_.ranges[n-2];
    intensity_derivative_[n-1] = scan_.intensities[n-2];
}

float ScanFeatureCalculator::variance(const boost::circular_buffer<float> &window)
{
    boost::circular_buffer<float>::const_iterator w_it;

    // mean
    float mean = 0;
    for (w_it = window.begin(); w_it != window.end(); ++w_it) {
        mean += *w_it;
    }
    mean /= window.size();

    // variance
    float var = 0;
    for (w_it = window.begin(); w_it != window.end(); ++w_it) {
        var += (*w_it - mean) * (*w_it - mean);
    }
    var /= window.size();

    return var;
}

void ScanFeatureCalculator::standardizeData(std::vector<float> &data) const
{
    // calculate mean and standard deviation (sd)
    float mean = std::accumulate(data.begin(), data.end(), 0) / data.size();
    float sd = 0;
    BOOST_FOREACH(float x, data) {
        sd += (x-mean)*(x-mean);
    }
    sd = sqrt( sd/data.size() );

    // standardize data by subtracting mean and dividing by standard deviation
    BOOST_FOREACH(float &x, data) {
        if (sd != 0)
            x = (x-mean)/sd;
        else
            x -= mean;
    }
}
