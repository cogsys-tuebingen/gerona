#ifndef OBSTACLEDETECTORPOLYGON_H
#define OBSTACLEDETECTORPOLYGON_H

#include "obstacledetector.h"
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>

class ObstacleDetectorPolygon : public ObstacleDetector
{
public:
    ObstacleDetectorPolygon():
        ObstacleDetector()
    {
        ros::param::param<std::string>("~obstacle_map_frame", map_frame_, "/laser");
    }

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
    virtual bool checkForObstacle(float width, float length, float course_angle, float curve_enlarge_factor) const;

    /**
     * @brief Get the polygon which is checked for obstacles.
     * @param width
     * @param length
     * @param course_angle
     * @param curve_enlarge_factor
     * @return Collision polygon.
     */
    virtual PolygonWithTfFrame getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const = 0;


private:
    tf::TransformListener tf_listener_;
    std::string map_frame_;


    /**
     * @brief Transform a cv::Point2f from an arbitrary TF-frame to map coordinates of the obstacle map.
     * @param p     The point which is to be transformed.
     * @param from  The TF-frame in which the point is defined.
     * @return      The transformed point in (non-integer!) map coordinates.
     */
    cv::Point2f transformPointToMap(const cv::Point2f &p, std::string from) const;

    //! Transform the given polygon to map coordinates.
    void transformPolygonToMap(PolygonWithTfFrame *polygon) const;
};

#endif // OBSTACLEDETECTORPOLYGON_H
