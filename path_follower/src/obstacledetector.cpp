#include "obstacledetector.h"

ObstacleDetector::ObstacleDetector()
{
}

void ObstacleDetector::gridMapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
}

bool ObstacleDetector::isObstacleAhead(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    if (!map_) {
        ROS_ERROR_THROTTLE(1, "ObstacleDetector: No map received.");
        return true;
    }

    return checkForObstacle(width, length, course_angle, curve_enlarge_factor);
}
