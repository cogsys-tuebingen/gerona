#ifndef OBSTACLEDETECTORACKERMANN_H
#define OBSTACLEDETECTORACKERMANN_H

#include "obstacledetector.h"

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
class ObstacleDetectorAckermann : public ObstacleDetector
{
public:
    ObstacleDetectorAckermann():
        ObstacleDetector()
    {}

protected:

    /**
     * @brief Check, if there is an obstacle within the obstacle box.
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkForObstacle(float width, float length, float course_angle, float curve_enlarge_factor) const;
};

#endif // OBSTACLEDETECTORACKERMANN_H
