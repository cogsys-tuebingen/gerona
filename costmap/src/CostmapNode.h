/*
 *  CostmapNode.h
 *
 *  This class demonstrates how to use the utility class Costmap in LibRobot
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef COSTMAPNODE_H
#define COSTMAPNODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <utils/LibRobot/Costmap.h>

class CostmapNode
{
public:
    CostmapNode(ros::NodeHandle &nh);

    void updateMap(const nav_msgs::OccupancyGridConstPtr &ptr);

private:
    Costmap costmap_;

    ros::Subscriber map_subscriber_;
    ros::Publisher map_publisher_;
};

#endif // COSTMAPNODE_H
