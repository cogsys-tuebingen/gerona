#ifndef MAPPROCESSOR_H
#define MAPPROCESSOR_H

#include <opencv2/core/core.hpp>
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

    /**
     * @brief Process the map and remove noise.
     * @param map The map that has to be processed. The call of this method will change the map!
     */
    void process(nav_msgs::OccupancyGrid *map);

    /**
     * @brief Checks if all points on the line from robot to goal are traversable.
     * @param map The map.
     * @param robot Position of the robot.
     * @param goal Position of the goal.
     * @return True if all points on the line between robot and goal are traversable, otherwise false.
     */
    static bool checkTraversabilityOfLine(const nav_msgs::OccupancyGrid &map, cv::Point2i robot,
                                          cv::Point2i goal);

private:
    //! Convert map to image.
    static void mapToImage(const nav_msgs::OccupancyGrid &map, cv::Mat1b *image);
    //! Convert image to map.
    static void imageToMap(const cv::Mat1b &image, nav_msgs::OccupancyGrid *map);
};

#endif // MAPPROCESSOR_H
