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

/**
 * example usage:
 *   rosrun costmap costmap_node _erode:=2 _dilate:=4 _sampling:=1
 */
class CostmapNode
{
public:
    CostmapNode(ros::NodeHandle &nh);

    void updateMap(const nav_msgs::OccupancyGridConstPtr &ptr);

private:
    ros::Subscriber map_subscriber_;
    ros::Publisher map_publisher_;

    int threshold_;
    int dilate_;
    int erode_;
    Costmap costmap_;
    std::vector<int8_t> map_data_;

    double running_avg_;
    int running_avg_ticks_;
    int sampling_;
};

#endif // COSTMAPNODE_H
