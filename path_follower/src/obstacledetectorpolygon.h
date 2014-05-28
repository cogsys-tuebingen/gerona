#ifndef OBSTACLEDETECTORPOLYGON_H
#define OBSTACLEDETECTORPOLYGON_H

#include "obstacledetector.h"
#include <opencv2/core/core.hpp>

class ObstacleDetectorPolygon : public ObstacleDetector
{
public:
    ObstacleDetectorPolygon():
        ObstacleDetector()
    {}

protected:
    /**
     * @brief Check, if there is an obstacle within the polygon, given by getPolygon().
     * @see getPolygon()
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkForObstacle(float width, float length, float course_angle, float curve_enlarge_factor) const;

    virtual std::vector<cv::Point> getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const = 0;
};

#endif // OBSTACLEDETECTORPOLYGON_H
