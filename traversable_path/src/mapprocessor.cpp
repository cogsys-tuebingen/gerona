#include "mapprocessor.h"
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "exceptions.h"

using namespace traversable_path;
using namespace Eigen;

MapProcessor::MapProcessor()
{
//    cv::namedWindow("map", CV_WINDOW_FREERATIO);
//    cv::namedWindow("processed map", CV_WINDOW_FREERATIO);
}

void MapProcessor::setMap(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;
    mapToImage(map, &map_img_);
}

void MapProcessor::mapToImage(const nav_msgs::OccupancyGrid &map, cv::Mat1b *image)
{
    // PLEASE NOTE: since cv::Mat is a matrix, here the row comes first.
    // This means the Point (x,y) can be accessed via image[y][x] (note the different order of x,y).
    *image = cv::Mat1b(map.info.height, map.info.width);

    for (size_t i = 0; i < map.data.size(); ++i) {
//        int row, col;
//        row = i / map.info.width;
//        col = i % map.info.width;
//        (*image)[row][col] = (map.data[i] == 0) ? 255 : 0;

        image->data[i] = (map.data[i] == 0) ? 255 : 0;
    }
}


void MapProcessor::imageToMap(const cv::Mat1b &img, nav_msgs::OccupancyGrid *map)
{
    map->info.width  = img.cols;
    map->info.height = img.rows;
    map->data.resize(img.cols * img.rows);

    for (size_t i = 0; i < map->data.size(); ++i) {
//        int row, col;
//        row = i / map->info.width;
//        col = i % map->info.width;
//        map->data[i] = img[row][col] == 0 ? 100 : 0;

        map->data[i] = img.data[i] == 0 ? 100 : 0;
    }
}

void MapProcessor::process(nav_msgs::OccupancyGrid *map)
{
    cv::Mat1b img;
    mapToImage(*map, &img);

	// First open the untraversable areas of the map to remove single untraversable cells. Then close to remove little holes in the untraversable areas.
	// Please note that OpenCV defines background as 0 and foreground as > 0. Thus opening the untraversable area (which is black) means closing the image and vice versa.
    cv::Mat kernel(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);
    kernel = cv::Mat(5, 5, CV_8U, cv::Scalar(1));
    cv::morphologyEx(img, img, cv::MORPH_OPEN, kernel);

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

bool MapProcessor::checkTraversabilityOfCircle(const Eigen::Vector2i &center, int radius) const
{
    // taken from http://free.pages.at/easyfilter/bresenham.html
    int x = -radius, y = 0, err = 2-2*radius; /* II. Quadrant */
    do {
        if (!(isPointTraversable(center + Vector2i(-x, +y))  /*   I. Quadrant */
           && isPointTraversable(center + Vector2i(-y, -x))  /*  II. Quadrant */
           && isPointTraversable(center + Vector2i(+x, -y))   /* III. Quadrant */
           && isPointTraversable(center + Vector2i(+y, +x)))) /*  IV. Quadrant */
        {
            return false;
        }

        radius = err;
        if (radius <= y) {
            err += ++y*2+1; /* e_xy+e_y < 0 */
        }
        if (radius > x || err > y) {
            err += ++x*2+1; /* e_xy+e_x > 0 or no 2nd y-step */
        }
    } while (x < 0);

    return true;
}

bool MapProcessor::isPointTraversable(const Eigen::Vector2i &point) const
{
    // Note: Point (x,y) is map_img_[y][x]
    if (point[1] < map_img_.rows && point[0] < map_img_.cols) {
        return (map_img_[point[1]][point[0]] != 0);
    } else {
        // points outside of the map are unknown and thus treated as untraversable
        return false;
    }
}

bool MapProcessor::checkGoalTraversability(const Eigen::Vector2f &robot, const Eigen::Vector2f &goal) const
{
    const float CHECK_GOAL_RADIUS = 0.1;

    // convert points to pixel coordinates of the map.
    Vector2i robot_on_map = transformToMap(robot);
    Vector2i goal_on_map = transformToMap(goal);

    return checkTraversabilityOfLine(robot_on_map, goal_on_map)
            && checkTraversabilityOfCircle(goal_on_map, CHECK_GOAL_RADIUS/map_.info.resolution);
}

Eigen::Vector2i MapProcessor::transformToMap(Eigen::Vector2f point) const
{
    Vector2i result;
    result[0] = (int) ((point[0] - map_.info.origin.position.x) / map_.info.resolution);
    result[1] = (int) ((point[1] - map_.info.origin.position.y) / map_.info.resolution);

    // cast map width/height from uint to int here, to avoid warnings. There will be no overflow problems since the
    // values will much less than 2*10^9.
    if (result[0] < 0 || result[0] >= (int)map_.info.width || result[1] < 0 || result[1] >= (int)map_.info.height) {
        throw TransformMapException();
    }

    return result;
}

size_t MapProcessor::transformToMapIndex(Eigen::Vector2f point) const
{
    Vector2i pixel = transformToMap(point);

    return pixel[1] * map_.info.width + pixel[0];
}
