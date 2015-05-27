#ifndef OBSTACLEDETECTORPATSY_H
#define OBSTACLEDETECTORPATSY_H

#include <path_follower/obstacle_avoidance/obstacledetectorpolygon.h>

/**
 * @brief Rectangle obstacle box that is enlarged and bend in curves
 *
 * See comments in the code (*.cpp) for details of the behaviour of the
 * box in curves.
 */
class ObstacleDetectorPatsy : public ObstacleDetectorPolygon
{
public:
    ObstacleDetectorPatsy(const tf::TransformListener *tf_listener):
        ObstacleDetectorPolygon(tf_listener)
    {}

protected:
    virtual PolygonWithTfFrame getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const;
};
#endif // OBSTACLEDETECTORPATSY_H
