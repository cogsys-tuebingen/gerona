/*
 *  CostmapNode.cpp
 *
 *  This class demonstrates how to use the utility class Costmap in LibRobot
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */
#include "Stopwatch.h"
#include "CostmapNode.h"

#define THRESHOLD 10
#define DILATE 6
#define ERODE 10

CostmapNode::CostmapNode(ros::NodeHandle &nh)
  : costmap_ (THRESHOLD, DILATE, ERODE)
{
  map_subscriber_ = nh.subscribe<nav_msgs::OccupancyGrid> ("/map", 10, boost::bind(&CostmapNode::updateMap, this, _1));
  map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> ("/map_inflated", 10);
}

void CostmapNode::updateMap(const nav_msgs::OccupancyGridConstPtr &ptr)
{
  Stopwatch timer;
  ros::Time start = ros::Time::now();
  map_data_.resize( ptr->info.width *ptr->info.height);
  timer.restart();
  costmap_.grow(ptr->data, ptr->info.width, ptr->info.height, map_data_);

  ros::Time stop = ros::Time::now();
  //ROS_INFO("costmap used %d msec",timer.msElapsed());
  ros::Duration diff = stop - start;
  //ROS_INFO_STREAM("map inflation took " << diff.toNSec() / 1000000. << "ms");

  nav_msgs::OccupancyGrid o;
  o.data = map_data_;
  o.info = ptr->info;

  map_publisher_.publish(o);
}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "costmap");
  ros::NodeHandle n("~");

  CostmapNode costmap(n);

  ros::spin();

  return 0;
}
