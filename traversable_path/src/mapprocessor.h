#ifndef MAPPROCESSOR_H
#define MAPPROCESSOR_H

#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>

class MapProcessor
{
public:
    MapProcessor();

    nav_msgs::OccupancyGrid process(const nav_msgs::OccupancyGrid &map);

private:
    nav_msgs::OccupancyGrid map_;

    cv::Mat1b mapToImage();
    void imageToMap(const cv::Mat1b &image);
};

#endif // MAPPROCESSOR_H
