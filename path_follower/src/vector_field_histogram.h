#ifndef VECTOR_FIELD_HISTOGRAM_H
#define VECTOR_FIELD_HISTOGRAM_H

/// SYSTEM
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>

class VectorFieldHistogram
{
public:
    VectorFieldHistogram();

    bool isReady();

    void setMap(const nav_msgs::OccupancyGrid& map);
    void create(double influence_distance, double threshold);
    void visualize(double course, double threshold);

    bool adjust(double course, double threshold, double &result_course);

private:
    void smooth();

private:
    int ang_res_;
    int s_max_;

    nav_msgs::OccupancyGrid map_;
    bool has_map_;

    std::vector<double> histogram_;
    std::vector<int> valley_;

    std::map<int, int> valley_begin_;
    std::map<int, int> valley_end_;

    std::map<int, int> valley_width_;
    std::map<int, bool> is_valley_wide_;

    int selected_valley_;
    int near_, far_;

    cv::Mat debug_map_;
};

#endif // VECTOR_FIELD_HISTOGRAM_H
