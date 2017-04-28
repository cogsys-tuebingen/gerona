#ifndef COLLISION_DETECTOR_OMNIDRIVE_H
#define COLLISION_DETECTOR_OMNIDRIVE_H

#include "collision_detector_polygon.h"

class CollisionDetectorOmnidrive : public CollisionDetectorPolygon
{
protected:
    virtual PolygonWithTfFrame getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const;
};

#endif // COLLISION_DETECTOR_OMNIDRIVE_H
