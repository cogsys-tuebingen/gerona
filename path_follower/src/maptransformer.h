#ifndef MAPTRANSFORMER_H
#define MAPTRANSFORMER_H

#include <string>
#include <opencv2/core/core.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

//! Simple helper class to transform points from and to map coordinates
class MapTransformer
{
public:
    void setMap(const nav_msgs::OccupancyGridConstPtr &map);

    /**
     * @brief Transform a cv::Point2f from an arbitrary TF-frame to map coordinates of the obstacle map.
     * @param p     The point which is to be transformed.
     * @param from  The TF-frame in which the point is defined.
     * @return      The transformed point in (non-integer!) map coordinates.
     * @throws tf::TransformException
     */
    cv::Point2f transformPointToMap(const cv::Point2f &p, std::string from) const;

    /**
     * @brief Transform a cv::Point2f from map coordinates of the obstacle map to an arbitrary TF-frame.
     * @param p     The point in map coordinates.
     * @param to    The target TF-frame in which the point is to be transformed.
     * @return      The transformed point.
     * @throws tf::TransformException
     */
    cv::Point2f transformPointFromMap(const cv::Point2f &p, std::string to) const;

private:
    nav_msgs::OccupancyGridConstPtr map_;

    tf::TransformListener tf_listener_;

    tf::Transform trans_from_map_cell_to_map_frame_;
};

#endif // MAPTRANSFORMER_H
