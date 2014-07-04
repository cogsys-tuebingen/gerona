#ifndef PATHLOOKOUT_H
#define PATHLOOKOUT_H

/// STL
#include <string>

/// THIRD PARTY
#include <opencv2/imgproc/imgproc.hpp>

/// ROS
#include <nav_msgs/OccupancyGrid.h>

/// PROJECT
#include "path.h"
#include "maptransformer.h"
#include "visualizer.h"

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
    MapTransformer map_trans_;

    nav_msgs::OccupancyGridConstPtr map_;

    cv::Mat map_image_;

    //Path path_;

    cv::Mat path_image_;

    Visualizer *visualizer_;

    //! draw the path to the path image
    /** @see path_image_ */
    void drawPathToImage(const Path &path);
};

#endif // PATHLOOKOUT_H
