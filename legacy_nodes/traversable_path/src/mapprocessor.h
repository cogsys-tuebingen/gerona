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

    /**
     * @brief Checks if the circle defined by center and radius is full traversable.
     * @param center Center of the circle on the map.
     * @param radius Radius of the circle (in map cells).
     * @return True if all points of the circle are traversable.
     */
    bool checkTraversabilityOfCircle(const Eigen::Vector2i &center, int radius) const;

    //! Checks if the specified point is traversable.
    bool isPointTraversable(const Eigen::Vector2i &point) const;

    /**
     * @brief Check if the goal (and the way to it) is traversable from the robots position.
     * @param robot Position of the robot in /map coordinates
     * @param goal Position of the goal in /map coordinates
     * @return True if the way to the goal is traversable.
     */
    bool checkGoalTraversability(const Eigen::Vector2f &robot, const Eigen::Vector2f &goal) const;

    /**
     * @brief Transform coordinates of a point to the map cell.
     * @param point Some point. Has to be in the same frame than the map (which is '/map').
     * @return Pixel coordinates on the map.
     * @throws traversable_path::TransformMapException if point lies outside of the map.
     */
    Eigen::Vector2i transformToMap(Eigen::Vector2f point) const;

    /**
     * @brief Transform coordinates of a point to the map cell index.
     *
     * This method returns the index of the map cell with which it can be accessed via map_.data[index].
     *
     * @param point Some point. Has to be in the same frame than the map (which is '/map').
     * @return Index of the map cell.
     * @throws traversable_path::TransformMapException if point lies outside of the map.
     */
    size_t transformToMapIndex(Eigen::Vector2f point) const;


    //! Convert map to image.
    static void mapToImage(const nav_msgs::OccupancyGrid &map, cv::Mat1b *image);
    //! Convert image to map.
    static void imageToMap(const cv::Mat1b &image, nav_msgs::OccupancyGrid *map);

private:
    //! The map.
    nav_msgs::OccupancyGrid map_;
    //! The map as image.
    cv::Mat1b map_img_;
};

#endif // MAPPROCESSOR_H
