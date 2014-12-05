#ifndef OBSTACLEDETECTORPOLYGON_H
#define OBSTACLEDETECTORPOLYGON_H

#include "obstacledetector.h"
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>

class ObstacleDetectorPolygon : public ObstacleDetector
{
protected:
    struct PolygonWithTfFrame
    {
        std::string frame;
        std::vector<cv::Point2f> polygon;
    };


    /**
     * @brief Check, if there is an obstacle within the polygon, given by getPolygon().
     * @see getPolygon()
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkOnCloud(ObstacleCloud::ConstPtr obstacles,
                              float width,
                              float length,
                              float course_angle,
                              float curve_enlarge_factor);


    /**
     * @brief Get the polygon which is checked for obstacles.
     * @return Collision polygon.
     */
    virtual PolygonWithTfFrame getPolygon(float width,
                                          float length,
                                          float course_angle,
                                          float curve_enlarge_factor) const = 0;


private:
    tf::TransformListener tf_listener_;

    //! Transform the given polygon to map coordinates.
    /** @throws tf::TransformException */
    ROS_DEPRECATED void transformPolygonToMap(PolygonWithTfFrame *polygon) const;

    void visualize(PolygonWithTfFrame polygon, bool hasObstacle) const;
};

#endif // OBSTACLEDETECTORPOLYGON_H
