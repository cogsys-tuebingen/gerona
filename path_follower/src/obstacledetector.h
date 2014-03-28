#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class ObstacleDetector
{
public:
    ObstacleDetector();

    void gridMapCallback(const nav_msgs::OccupancyGridConstPtr &map);

    /**
     * @brief Check, if there is an obstacle within the obstacle box.
     * @return True if there is an obstacle, false if not.
     */
    bool isObstacleAhead(float width, float length, float steering_angle, float curve_enlarge_factor) const;

private:
    static const char FREE = 0;
    static const char OCCUPIED = 100;

    nav_msgs::OccupancyGridConstPtr map_;
};

#endif // OBSTACLEDETECTOR_H
