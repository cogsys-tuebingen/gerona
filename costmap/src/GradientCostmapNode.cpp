/*
 *  GradientCostmapNode.cpp
 *
 *  Created on: Mar 11, 2014
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#include <utils_general/Stopwatch.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>

class GradientCostmapNode
{
public:

    GradientCostmapNode(ros::NodeHandle &nh)
        : running_avg_(0), running_avg_ticks_(0)
    {
        std::string map_topic ("/map");
        std::string map_service ("/dynamic_map");
        std::string map_topic_result ("/map/cost");
        std::string map_service_result ("/dynamic_map/cost");
        nh.param("topic_map", map_topic, map_topic);
        nh.param("map_service", map_service, map_service);
        nh.param("topic_map_result", map_topic_result, map_topic_result);
        nh.param("map_service_result", map_service_result, map_service_result);

        nh.param("max_distance_meters", max_distance_meters_, 2.0);
        nh.param("scale", scale_, 100.0);

        map_subscriber_ = nh.subscribe<nav_msgs::OccupancyGrid> (map_topic, 10, boost::bind(&GradientCostmapNode::updateMapCallback, this, _1));
        map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> (map_topic_result, 10, true);


        map_service_client = nh.serviceClient<nav_msgs::GetMap> (map_service);
        map_service_ = nh.advertiseService (map_service_result, &GradientCostmapNode::getMap, this);
    }

    bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
    {
        nav_msgs::GetMap map_service;
        if(map_service_client.call(map_service)) {
            updateMap(map_service.response.map);
            res.map = current_map_;
            return true;
        } else {
            return false;
        }
    }

    void updateMapCallback(const nav_msgs::OccupancyGridConstPtr &ptr)
    {
	ROS_WARN("received map");
	ROS_WARN_STREAM("SIZE is " << ptr->info.width << " x " << ptr->info.height);
        updateMap(*ptr);

        map_publisher_.publish(current_map_);
    }

    void updateMap(const nav_msgs::OccupancyGrid &map)
    {
        map_data_ = map.data;

        Stopwatch timer;

        cv::Mat working(map.info.height, map.info.width, CV_8UC1, map_data_.data());

        cv::Mat unknown_mask;
        cv::inRange(working, 101, 255, unknown_mask);
        working.setTo(0, unknown_mask);
        cv::threshold(working, working, 50, 255, cv::THRESH_BINARY);

        working = 255 - working;

        cv::Mat distance;
        cv::distanceTransform(working, distance, CV_DIST_L2, CV_DIST_MASK_PRECISE);

        distance *= 3.0;

        distance.convertTo(working, CV_8UC1, (scale_ * map.info.resolution / max_distance_meters_));

        cv::threshold(working, working, 254, 254, CV_THRESH_TRUNC);
        working = 254 - working;
        working.setTo(255, unknown_mask);


        current_map_.data = map_data_;
        current_map_.info = map.info;
        current_map_.header = map.header;

        double diff =  timer.elapsed() * 1000;
        running_avg_ticks_++;
        running_avg_ = (running_avg_ * (running_avg_ticks_-1) / running_avg_ticks_) + diff / running_avg_ticks_;
        ROS_INFO_STREAM("map inflation took " << diff << "ms , sampling: " << sampling_ << "] [avg. " << running_avg_ << "ms]");
    }


private:
    ros::Subscriber map_subscriber_;
    ros::Publisher map_publisher_;

    ros::ServiceClient map_service_client;
    ros::ServiceServer map_service_;

    nav_msgs::OccupancyGrid current_map_;
    std::vector<int8_t> map_data_;

    double running_avg_;
    int running_avg_ticks_;
    int sampling_;

    double scale_;
    double max_distance_meters_;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "costmap");
    ros::NodeHandle n("~");

    GradientCostmapNode costmap(n);

    ros::spin();

    return 0;
}
