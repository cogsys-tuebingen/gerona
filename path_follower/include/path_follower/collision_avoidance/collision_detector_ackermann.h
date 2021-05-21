#ifndef COLLISION_DETECTOR_ACKERMANN_H
#define COLLISION_DETECTOR_ACKERMANN_H

#include "collision_detector_polygon.h"

/**
 * @brief Rectangle obstacle box that is enlarged and bend in curves
 *
 * See comments in the code (*.cpp) for details of the behaviour of the
 * box in curves.
 */
class CollisionDetectorAckermann : public CollisionDetectorPolygon
{
public:
    virtual bool avoid(MoveCommand * const cmd, const CollisionAvoider::State &state) override;

protected:
    PolygonWithTfFrame getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const override;

    float velocity_;
};

#endif // COLLISION_DETECTOR_ACKERMANN_H
