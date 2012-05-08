#include "mapprocessor.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

MapProcessor::MapProcessor()
{
    cv::namedWindow("map", CV_WINDOW_FREERATIO);
    cv::namedWindow("processed map", CV_WINDOW_FREERATIO);
}

cv::Mat1b MapProcessor::mapToImage()
{
    cv::Mat1b img(map_.info.width, map_.info.height);

    for (size_t i = 0; i < map_.data.size(); ++i) {
        int row, col;
        row = i % map_.info.width;
        col = i / map_.info.width;

        img[row][col] = (map_.data[i] == 0) ? 255 : 0;
    }

    return img;
}


void MapProcessor::imageToMap(const cv::Mat1b &img)
{
    for (size_t i = 0; i < map_.data.size(); ++i) {
        int row, col;
        row = i % map_.info.width;
        col = i / map_.info.width;

        map_.data[i] = img[row][col] == 0 ? 100 : 0;
    }
}

nav_msgs::OccupancyGrid MapProcessor::process(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;

    cv::Mat1b img = mapToImage();
    //cv::Mat1b eroded;
    cv::Mat1b processed;

    //cv::erode(img, eroded, cv::Mat());
    //cv::dilate(img, dilated, cv::Mat());

    cv::Mat foo(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(img, processed, cv::MORPH_CLOSE, foo);

//    cv::imshow("map", img);
//    cv::imshow("processed map", processed);
//    cv::waitKey(3);

    imageToMap(processed);

    return map_;
}
