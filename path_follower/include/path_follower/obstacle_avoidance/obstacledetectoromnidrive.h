#ifndef OBSTACLEDETECTOROMNIDRIVE_H
#define OBSTACLEDETECTOROMNIDRIVE_H

#include "obstacledetectorpolygon.h"

class ObstacleDetectorOmnidrive : public ObstacleDetectorPolygon
{
protected:
    virtual PolygonWithTfFrame getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const;
};

#endif // OBSTACLEDETECTOROMNIDRIVE_H
