#ifndef PATHLOOKOUT_H
#define PATHLOOKOUT_H

/// STL
#include <string>

/// THIRD PARTY
#include <opencv2/core/core.hpp>

/// ROS
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

/// PROJECT
#include "path.h"

class PathLookout
{
public:
    PathLookout();

    void setMap(const nav_msgs::OccupancyGridConstPtr &msg);

    //! Set the path, which is to be checked for obstacles.
    void setPath(const Path &path);

    //! Check if there is an obstacle on the path ahead of the robot, that gives a reason to cancel the current path.
    /**
     * @return True, if there is an obstacle that justifies the abortion of the path.
     */
    bool lookForObstacles();

private:
    tf::TransformListener tf_listener_;

    nav_msgs::OccupancyGridConstPtr map_;

    cv::Mat map_image_;

    //Path path_;

    cv::Mat path_image_;

    //! draw the path to the path image
    /** @see path_image_ */
    void drawPathToImage(const Path &path);
    cv::Point2f transformPointToMap(const cv::Point2f &p, std::string from) const;
};

#endif // PATHLOOKOUT_H
