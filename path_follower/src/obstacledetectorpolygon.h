#ifndef OBSTACLEDETECTORPOLYGON_H
#define OBSTACLEDETECTORPOLYGON_H

#include "obstacledetector.h"
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "maptransformer.h"

class ObstacleDetectorPolygon : public ObstacleDetector
{
public:
    ObstacleDetectorPolygon():
        ObstacleDetector()
    {
    }

    virtual void setMap(const nav_msgs::OccupancyGridConstPtr &map);

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
    virtual bool checkOnMap(float width, float length, float course_angle, float curve_enlarge_factor);

    virtual bool checkOnScan(const sensor_msgs::LaserScanConstPtr &scan, float width, float length, float course_angle, float curve_enlarge_factor);

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
    MapTransformer map_trans_;
    tf::TransformListener tf_listener_;
    laser_geometry::LaserProjection laser_projector_;

    //! Transform the given polygon to map coordinates.
    /** @throws tf::TransformException */
    void transformPolygonToMap(PolygonWithTfFrame *polygon) const;

    void transformPolygonToScan(PolygonWithTfFrame *polygon) const;

    void visualize(PolygonWithTfFrame polygon, bool hasObstacle) const;

    //bool checkPoint()
};

#endif // OBSTACLEDETECTORPOLYGON_H
