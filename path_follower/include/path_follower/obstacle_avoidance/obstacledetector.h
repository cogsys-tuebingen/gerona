#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

#include <path_follower/obstacle_avoidance/obstacleavoider.h>

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <path_follower/utils/parameters.h>

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
class ObstacleDetector: public ObstacleAvoider
{
public:
    ObstacleDetector();

    virtual void avoid(tf::Vector3 * const cmd, const ObstacleCloud &obstacles, const State &state);

    //! Callback for the obstacle map. Make sure, that the map is binary!
    ROS_DEPRECATED virtual void setMap(const nav_msgs::OccupancyGridConstPtr &map);
    //! Callback for the laser scan
    ROS_DEPRECATED virtual void setScan(const sensor_msgs::LaserScanConstPtr &scan, bool isBack=false);

    ROS_DEPRECATED virtual void setUseMap(bool use);
    ROS_DEPRECATED virtual void setUseScan(bool use);

    /**
     * @brief Check, if there is an obstacle in front of the robot.
     *
     * @param width Width of the collision box.
     * @param length Length of the collision box. If an object is within this distance, an collision is thrown.
     * @param course_angle Angle of the current course (e.g. use steering angle).
     * @param curve_enlarge_factor The width of the box is enlarged a bit in curves. This argument controls how much (it is misleadingly called 'length' in LaserEnvironment).
     * @return True, if there is an object within the collision box.
     */
    ROS_DEPRECATED bool isObstacleAhead(float width, float length, float course_angle, float curve_enlarge_factor);

protected:
    //! Value of the obstacle map for free cells.
    static const char FREE = 0;
    //! Value of the obstacle map for occupied cells.
    static const char OCCUPIED = 100;

    //! The current obstacle map.
    nav_msgs::OccupancyGridConstPtr map_;

    /**
     * @brief Check, if there is an obstacle in the map within the obstacle box.
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkOnMap(float width, float length, float course_angle, float curve_enlarge_factor) = 0;

    /**
     * @brief Check, if there is an scan point within the obstacle box.
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkOnScan(const sensor_msgs::LaserScanConstPtr &scan, float width, float length, float course_angle, float curve_enlarge_factor) = 0;

private:
    bool use_map_;
    bool use_scan_;

    //! The current laser scan
    sensor_msgs::LaserScanConstPtr scan_;
    sensor_msgs::LaserScanConstPtr scan_back_;
};

#endif // OBSTACLEDETECTOR_H
