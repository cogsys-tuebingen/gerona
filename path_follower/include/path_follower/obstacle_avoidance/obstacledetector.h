#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

#include <path_follower/obstacle_avoidance/obstacleavoider.h>

/**
 * @brief Simple 'emergency break' obstacle avoider that stops when an obstacle is within a
 *        defined 'obstacle box' in front of the robot.
 *
 * This is the most simple obstacle avoider which should be feasible for all robot models. It is
 * therefore a good fallback which can always be used, when there is no more intelligent
 * obstacle avoider.
 * When checking for obstacles, a box in front of the robot is calculated. If one or more
 * obstacles are within this box, the robot is stopped immediately by setting the velocity of
 * the move command to zero. There are no attempts to get around the obstacle.
 * There are several child classes which implement different shapes of obstacle boxes
 *
 *
 * In courves, the box is bend toward the direction of the path. For more details on this, see the comments inside
 * the method isObstacleAhead().
 */
class ObstacleDetector: public ObstacleAvoider
{
public:
    virtual bool avoid(tf::Vector3 * const cmd,
                       ObstacleCloud::ConstPtr obstacles,
                       const State &state);

protected:
    /**
     * @brief Check, if there is an obstacle within the obstacle box in front of the robot.
     *
     * @param obstacles Point cloud with points for detected obstacles.
     * @param width Width of the collision box.
     * @param length Length of the collision box. If an object is within this distance, an
     *               collision is thrown.
     * @param course_angle Angle of the current course (e.g. use steering angle).
     * @param curve_enlarge_factor The width of the box is enlarged a bit in curves. This
     *                             argument controls how much (it is misleadingly called
     *                             'length' in LaserEnvironment).
     * @return True, if there is an object within the collision box.
     */
    virtual bool checkOnCloud(ObstacleCloud::ConstPtr obstacles,
                              float width,
                              float length,
                              float course_angle,
                              float curve_enlarge_factor) = 0;
};

#endif // OBSTACLEDETECTOR_H
