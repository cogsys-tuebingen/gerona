#include "mapprocessor.h"
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

MapProcessor::MapProcessor()
{
//    cv::namedWindow("map", CV_WINDOW_FREERATIO);
//    cv::namedWindow("processed map", CV_WINDOW_FREERATIO);
}

void MapProcessor::setMap(const nav_msgs::OccupancyGrid &map)
{
    mapToImage(map, &map_img_);
}

void MapProcessor::mapToImage(const nav_msgs::OccupancyGrid &map, cv::Mat1b *image)
{
    // PLEASE NOTE: since cv::Mat is a matrix, here the row comes first.
    // This means the Point (x,y) can be accessed via image[y][x] (note the different order of x,y).
    *image = cv::Mat1b(map.info.height, map.info.width);

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

    cv::Mat kernel(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

//    cv::imshow("map", img);
//    cv::imshow("processed map", processed);
//    cv::waitKey(3);

    imageToMap(img, map);
}

cv::LineIterator MapProcessor::getLineIterator(const cv::Point2i &p1, const cv::Point2i &p2) const
{
    return cv::LineIterator(map_img_, p1, p2);
}

cv::LineIterator MapProcessor::getLineIterator(const Eigen::Vector2i &p1, const Eigen::Vector2i &p2) const
{
    return getLineIterator(cv::Point2i(p1[0], p1[1]), cv::Point2i(p2[0], p2[1]));
}

bool MapProcessor::checkTraversabilityOfLine(const cv::Point2i &robot, const cv::Point2i &goal) const
{
    cv::LineIterator line_it = getLineIterator(robot, goal);
    for (int i = 0; i < line_it.count; ++i, ++line_it) {
        if (*((uchar*) *line_it) == 0) {
            return false;
        }
    }
    return true;
}

bool MapProcessor::checkTraversabilityOfLine(const Eigen::Vector2i &robot, const Eigen::Vector2i &goal) const
{
    return checkTraversabilityOfLine(cv::Point2i(robot[0], robot[1]), cv::Point2i(goal[0], goal[1]));
}
