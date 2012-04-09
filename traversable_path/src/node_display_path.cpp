#include <ros/ros.h>
#include "traversable_path/LaserScanClassification.h"
#include "visualization.h"

Visualization g_visualizer;

void classificationCallback(traversable_path::LaserScanClassificationConstPtr classification)
{
    g_visualizer.paintPath(classification->traversable);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "display_path");
    ros::NodeHandle node_handle;

    // subscribe for scan classification
    ros::Subscriber subscribe_classification = node_handle.subscribe("traversability", 1, &classificationCallback);

    ros::spin();
    return 1;
}
