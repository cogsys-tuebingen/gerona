#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

/**
 * @brief Checks for obstacles in front of the robot, using an obstacle map.
 *
 * For the obstacle detection, an up-to-date obstacle map is required, which can be set, using gridMapCallback (e.g.
 * by directly setting this method as callback for a subscriber.
 * The map has to be binary, i.e. each cell is either "free" or "occupied".
 * The robot is assumed to be in the origin of the map.
 *
 * When checking for obstacles, a box in front of the robot is calculated. An obstacle is recognized, if there is one
 * (or more) occupied cell inside this box.
 *
 * In courves, the box is bend toward the direction of the path. For more details on this, see the comments inside
 * the method isObstacleAhead().
 */
class ObstacleDetector
{
public:
    ObstacleDetector();

    //! Callback for the obstacle map. Make sure, that the map is binary!
    virtual void setMap(const nav_msgs::OccupancyGridConstPtr &map);
    //! Callback for the laser scan
    virtual void setScan(const sensor_msgs::LaserScanConstPtr &scan);

    virtual void setUseMap(bool use);
    virtual void setUseScan(bool use);

    /**
     * @brief Check, if there is an obstacle in front of the robot.
     * @return True if there is an obstacle, false if not.
     */
    bool isObstacleAhead(float width, float length, float course_angle, float curve_enlarge_factor);

protected:
    //! Value of the obstacle map for free cells.
    static const char FREE = 0;
    //! Value of the obstacle map for occupied cells.
    static const char OCCUPIED = 100;

    //! The current obstacle map.
    nav_msgs::OccupancyGridConstPtr map_;

    //! The current laser scan
    sensor_msgs::LaserScanConstPtr scan_;

    /**
     * @brief Check, if there is an obstacle in the map within the obstacle box.
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkOnMap(float width, float length, float course_angle, float curve_enlarge_factor) = 0;

    /**
     * @brief Check, if there is an scan point within the obstacle box.
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkOnScan(float width, float length, float course_angle, float curve_enlarge_factor) = 0;

private:
    bool use_map_;
    bool use_scan_;
};

#endif // OBSTACLEDETECTOR_H
