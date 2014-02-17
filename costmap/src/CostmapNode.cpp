/*
 *  CostmapNode.cpp
 *
 *  This class demonstrates how to use the utility class Costmap in LibRobot
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include <utils_general/Stopwatch.h>
#include "CostmapNode.h"

CostmapNode::CostmapNode(ros::NodeHandle &nh)
  : running_avg_(0), running_avg_ticks_(0)
{
  nh.param<int> ("threshold", threshold_, 10);
  nh.param<int> ("dilate", dilate_, 4);
  nh.param<int> ("erode", erode_, 4);
  nh.param<int> ("sampling", sampling_, 1);

  std::string map_topic ("/map");
  std::string map_topic_result ("/map/inflated");
  std::string map_service ("/dynamic_map/inflated");
  nh.param("topic_map", map_topic, map_topic);
  nh.param("topic_map_result", map_topic_result, map_topic_result);
  nh.param("service_map", map_service, map_service);

  map_subscriber_ = nh.subscribe<nav_msgs::OccupancyGrid> (map_topic, 10, boost::bind(&CostmapNode::updateMapCallback, this, _1));
  map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> (map_topic_result, 10);


  map_service_client = nh.serviceClient<nav_msgs::GetMap> ("/dynamic_map");
  map_service_ = nh.advertiseService (map_service, &CostmapNode::getMap, this);
}

bool CostmapNode::getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
  nav_msgs::GetMap map_service;
  map_service_client.call(map_service);
  updateMap(map_service.response.map);

  res.map = current_map_;
  return true;
}

void CostmapNode::updateMapCallback(const nav_msgs::OccupancyGridConstPtr &ptr)
{
  updateMap(*ptr);
}

void CostmapNode::updateMap(const nav_msgs::OccupancyGrid &map)
  {
  map_data_.resize( map.info.width *map.info.height);

  Stopwatch timer;
  costmap_.grow(map.data, map.info.width, map.info.height,
                map_data_,
                dilate_, erode_, threshold_, sampling_);

  double diff =  timer.elapsed() * 1000;
  running_avg_ticks_++;
  running_avg_ = (running_avg_ * (running_avg_ticks_-1) / running_avg_ticks_) + diff / running_avg_ticks_;
//  ROS_INFO_STREAM("map inflation took " << diff << "ms [dilate: " << dilate_ << ", erode: " << erode_ <<
 //                 ", sampling: " << sampling_ << "] [avg. " << running_avg_ << "ms]");


  current_map_.data = map_data_;
  current_map_.info = map.info;

  map_publisher_.publish(current_map_);
}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "costmap");
  ros::NodeHandle n("~");

  CostmapNode costmap(n);

  ros::spin();

  return 0;
}
