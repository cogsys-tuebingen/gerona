#include "obstacledetector.h"

ObstacleDetector::ObstacleDetector():
    use_map_(true),
    use_scan_(true)
{
}

void ObstacleDetector::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
}

void ObstacleDetector::setScan(const sensor_msgs::LaserScanConstPtr &scan)
{
    scan_ = scan;
}

void ObstacleDetector::setUseMap(bool use)
{
    use_map_ = use;
}

void ObstacleDetector::setUseScan(bool use)
{
    use_scan_ = use;
}

bool ObstacleDetector::isObstacleAhead(float width, float length, float course_angle, float curve_enlarge_factor)
{
    // if there is neither map nor scan, be save and assume there is an obstacle, otherwise init with false.
    bool obstacle = !map_ && !scan_;

    if (use_map_) {
        if (!map_) {
            ROS_ERROR_THROTTLE(1, "ObstacleDetector: No map received.");
        }
        obstacle |= checkOnMap(width, length, course_angle, curve_enlarge_factor);
    }

    if (use_scan_) {
        if (!scan_) {
            ROS_ERROR_THROTTLE(1, "ObstacleDetector: No scan received.");
            return true;
        }
        obstacle |= checkOnScan(width, length, course_angle, curve_enlarge_factor);
    }

    return obstacle;
}
