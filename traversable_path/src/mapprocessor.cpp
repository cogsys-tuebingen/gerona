#include "mapprocessor.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

MapProcessor::MapProcessor()
{
//    cv::namedWindow("map", CV_WINDOW_FREERATIO);
//    cv::namedWindow("processed map", CV_WINDOW_FREERATIO);
}

void MapProcessor::mapToImage(const nav_msgs::OccupancyGrid &map, cv::Mat1b *image)
{
    // PLEASE NOTE: since cv::Mat is a matrix, here the row comes first.
    // This means the Point (x,y) can be accessed via image[y][x] (note the different order of x,y).
    *image = cv::Mat1b(map.info.height, map.info.width);

    ROS_INFO("map: %dx%d", map.info.width, map.info.height);
    ROS_INFO("img: %dx%d", image->cols, image->rows);

    for (size_t i = 0; i < map.data.size(); ++i) {
        int row, col;
        row = i / map.info.width;
        col = i % map.info.width;

        (*image)[row][col] = (map.data[i] == 0) ? 255 : 0;
    }
}


void MapProcessor::imageToMap(const cv::Mat1b &img, nav_msgs::OccupancyGrid *map)
{
    map->info.width  = img.cols;
    map->info.height = img.rows;
    map->data.resize(img.cols * img.rows);

    for (size_t i = 0; i < map->data.size(); ++i) {
        int row, col;
        row = i / map->info.width;
        col = i % map->info.width;

        map->data[i] = img[row][col] == 0 ? 100 : 0;
    }
}

void MapProcessor::process(nav_msgs::OccupancyGrid *map)
{
    cv::Mat1b img;
    mapToImage(*map, &img);

    cv::Mat foo(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, foo);

//    cv::imshow("map", img);
//    cv::imshow("processed map", processed);
//    cv::waitKey(3);

    imageToMap(img, map);
}

bool MapProcessor::checkTraversabilityOfLine(const nav_msgs::OccupancyGrid &map, cv::Point2i robot, cv::Point2i goal)
{
    cv::Mat1b img;
    mapToImage(map, &img);

    cv::LineIterator line_it(img, robot, goal, 8);
    for (int i = 0; i < line_it.count; ++i, ++line_it) {
        if (*line_it == 0) {
            cv::imshow("line", img); cv::waitKey(3);
            return false;
        }
        img.at<uint8_t>(line_it.pos()) = 0;
    }
    cv::imshow("line", img); cv::waitKey(3);
    return true;
}
