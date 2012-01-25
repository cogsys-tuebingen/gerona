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

CostmapNode::CostmapNode(ros::NodeHandle &nh)
  : running_avg_(0), running_avg_ticks_(0)
{
  nh.param<int> ("threshold", threshold_, 10);
  nh.param<int> ("dilate", dilate_, 4);
  nh.param<int> ("erode", erode_, 4);
  nh.param<int> ("sampling", sampling_, 1);

  std::string map ("/map");
  std::string map_res ("/map_inflated");
  nh.param<std::string> ("topic_mac", map, map);
  nh.param<std::string> ("topic_mac_result", map_res, map_res);

  map_subscriber_ = nh.subscribe<nav_msgs::OccupancyGrid> (map, 10, boost::bind(&CostmapNode::updateMap, this, _1));
  map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> (map_res, 10);
}

void CostmapNode::updateMap(const nav_msgs::OccupancyGridConstPtr &ptr)
{
  map_data_.resize( ptr->info.width *ptr->info.height);

  Stopwatch timer;
  costmap_.grow(ptr->data, ptr->info.width, ptr->info.height,
                map_data_,
                dilate_, erode_, threshold_, sampling_);

  double diff =  timer.elapsed() * 1000;
  running_avg_ticks_++;
  running_avg_ = (running_avg_ * (running_avg_ticks_-1) / running_avg_ticks_) + diff / running_avg_ticks_;
//  ROS_INFO_STREAM("map inflation took " << diff << "ms [dilate: " << dilate_ << ", erode: " << erode_ <<
 //                 ", sampling: " << sampling_ << "] [avg. " << running_avg_ << "ms]");

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
