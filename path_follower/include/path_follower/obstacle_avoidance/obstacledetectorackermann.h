#ifndef OBSTACLEDETECTORACKERMANN_H
#define OBSTACLEDETECTORACKERMANN_H

#include <path_follower/obstacle_avoidance/obstacledetectorpolygon.h>

/**
 * @brief Rectangle obstacle box that is enlarged and bend in curves
 *
 * See comments in the code (*.cpp) for details of the behaviour of the
 * box in curves.
 */
class ObstacleDetectorAckermann : public ObstacleDetectorPolygon
{
public:
    virtual bool avoid(MoveCommand * const cmd, const ObstacleAvoider::State &state) override;

protected:
    virtual PolygonWithTfFrame getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const;

    float velocity_;
};

#endif // OBSTACLEDETECTORACKERMANN_H
