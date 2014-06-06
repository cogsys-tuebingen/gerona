#include "obstacledetectoromnidrive.h"

ObstacleDetectorPolygon::PolygonWithTfFrame ObstacleDetectorOmnidrive::getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    PolygonWithTfFrame pwf;
    pwf.frame = "/base_link";

    pwf.polygon.push_back(cv::Point2f(1,1));
    pwf.polygon.push_back(cv::Point2f(1.5,0));
    pwf.polygon.push_back(cv::Point2f(1,-1));
    pwf.polygon.push_back(cv::Point2f(0,-1.5));
    pwf.polygon.push_back(cv::Point2f(-1,-1));
    pwf.polygon.push_back(cv::Point2f(-1.5,0));
    pwf.polygon.push_back(cv::Point2f(-1,1));
    pwf.polygon.push_back(cv::Point2f(0,1.5));

    return pwf;
}
