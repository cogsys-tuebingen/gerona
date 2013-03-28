/**
 * @brief Main file of the node <i>classify_terrain</i>
 *
 * @author Felix Widmaier
 * @version $Id$
 */

#include <ros/ros.h>
#include "terrainclassifier.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "classify_terrain");

    TerrainClassifier dls;

    // main loop
    ros::spin();
    return 0;
}
