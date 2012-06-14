#ifndef MAPPROCESSOR_H
#define MAPPROCESSOR_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>

/**
 * @brief Cleans an traversability map from noise.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class MapProcessor
{
public:
    MapProcessor();


    void setMap(const nav_msgs::OccupancyGrid &map);

    /**
     * @brief Process the map and remove noise.
     * @param map The map that has to be processed. The call of this method will change the map!
     */
    void process(nav_msgs::OccupancyGrid *map);

    /**
     * @brief getLineIterator
     *
     * Please note that the values of the map image are different from those of the map. While on the map 0 is
     * traversable and 100 marks an obstacle, on the image traversable cells have the value 255 and untraversable
     * once 0.
     *
     * @param p1
     * @param p2
     * @return
     */
    cv::LineIterator getLineIterator(const cv::Point2i &p1, const cv::Point2i &p2) const;
    cv::LineIterator getLineIterator(const Eigen::Vector2i &p1, const Eigen::Vector2i &p2) const;

    /**
     * @brief Checks if all points on the line from robot to goal are traversable.
     * @param map The map.
     * @param robot Position of the robot.
     * @param goal Position of the goal.
     * @return True if all points on the line between robot and goal are traversable, otherwise false.
     */
    bool checkTraversabilityOfLine(const cv::Point2i &robot, const cv::Point2i &goal) const;
    bool checkTraversabilityOfLine(const Eigen::Vector2i &robot, const Eigen::Vector2i &goal) const;


    //! Convert map to image.
    static void mapToImage(const nav_msgs::OccupancyGrid &map, cv::Mat1b *image);
    //! Convert image to map.
    static void imageToMap(const cv::Mat1b &image, nav_msgs::OccupancyGrid *map);

private:
    //! The occupancy grid as image.
    cv::Mat1b map_img_;
};

#endif // MAPPROCESSOR_H
