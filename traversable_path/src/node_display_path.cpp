/**
 * @brief Node that visualizes the path.
 *
 * This node is very simple. It subscribes for the terrain classification and sends the traversable-vector to
 * Visualization::paintPath().
 *
 * @author Felix Widmaier
 * @version $Id$
 * @see Visualization
 */
#include <ros/ros.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include "point_types.h"
#include "visualization.h"

Visualization g_visualizer;

//! Callback
void classificationCallback(const pcl::PointCloud<PointXYZRGBT>::ConstPtr &classification)
{
    g_visualizer.paintPath(classification);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "display_path");
    ros::NodeHandle node_handle;

    // subscribe for scan classification
    ros::Subscriber subscribe_classification = node_handle.subscribe("path_classification_cloud", 1,
                                                                     &classificationCallback);

    ros::spin();
    return 0;
}
