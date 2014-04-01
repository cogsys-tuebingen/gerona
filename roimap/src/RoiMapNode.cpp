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

class ROIMapNode
{
public:

    ROIMapNode(ros::NodeHandle &nh)
        : running_avg_(0), running_avg_ticks_(0)
    {
        std::string map_topic ("/map/hector");
        std::string map_service ("/dynamic_map/hector");
        std::string map_topic_result ("/map");
        std::string map_service_result ("/dynamic_map");
        nh.param("topic_map", map_topic, map_topic);
        nh.param("map_service", map_service, map_service);
        nh.param("topic_map_result", map_topic_result, map_topic_result);
        nh.param("map_service_result", map_service_result, map_service_result);

        nh.param("padding", padding_, 0.0);

        map_subscriber_ = nh.subscribe<nav_msgs::OccupancyGrid> (map_topic, 10, boost::bind(&ROIMapNode::updateMapCallback, this, _1));
        map_publisher_  = nh.advertise<nav_msgs::OccupancyGrid> (map_topic_result, 10, true);


        map_service_client = nh.serviceClient<nav_msgs::GetMap> (map_service);
        map_service_ = nh.advertiseService (map_service_result, &ROIMapNode::getMap, this);
    }

    bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
    {
        nav_msgs::GetMap map_service;
        map_service_client.call(map_service);
        updateMap(map_service.response.map);

        res.map = current_map_;
        return true;
    }

    void updateMapCallback(const nav_msgs::OccupancyGridConstPtr &ptr)
    {
        updateMap(*ptr);

        map_publisher_.publish(current_map_);
    }

    void updateMap(const nav_msgs::OccupancyGrid &map)
    {
        map_data_ = map.data;

        Stopwatch timer;

        cv::Mat working(map.info.height, map.info.width, CV_8SC1, map_data_.data());
        cv::Point min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
        cv::Point max(-1,-1);

        int8_t* ptr = working.ptr<int8_t>();
        for(int y = 0 ; y < working.rows ; ++y) {
            for(int x = 0 ; x < working.cols ; ++x) {
                if(ptr[y * working.cols + x] != -1) {
                   min.x = std::min(min.x, x);
                   min.y = std::min(min.y, y);
                   max.x = std::max(max.x, x);
                   max.y = std::max(max.y, y);
                }
            }
        }


        int padding = std::floor(padding_ / map.info.resolution + 0.5);
        min.x = std::max(min.x - padding, 0);
        min.y = std::max(min.y - padding, 0);
        max.x = std::min(max.x + padding, working.cols - 1);
        max.y = std::min(max.y + padding, working.rows - 1);

        int width  = max.x - min.x;
        int height = max.y - min.y;
        cv::Rect roi(min.x , min.y, width, height);

        current_map_.data.resize(width * height);
        int8_t *data_ptr = current_map_.data.data();
        for(int y = 0 ; y < height ; ++y) {
            for(int x = 0 ; x < width ; ++x) {
                 data_ptr[y * width + x] = ptr[(min.y + y) * working.cols + (min.x + x)];
            }
        }

        current_map_.info                    = map.info;
        current_map_.info.height             = height;
        current_map_.info.width              = width;
        current_map_.info.origin.position.x += roi.x * map.info.resolution;
        current_map_.info.origin.position.y += roi.y * map.info.resolution;

        double diff =  timer.elapsed() * 1000;
        running_avg_ticks_++;
        running_avg_ = (running_avg_ * (running_avg_ticks_-1) / running_avg_ticks_) + diff / running_avg_ticks_;
        std::cout << "map shrink took " << diff << "ms , sampling: " << sampling_ << "] [avg. " << running_avg_ << "ms]" << std::endl;
    }


private:
    ros::Subscriber     map_subscriber_;
    ros::Publisher      map_publisher_;

    ros::ServiceClient  map_service_client;
    ros::ServiceServer  map_service_;

    nav_msgs::OccupancyGrid current_map_;
    std::vector<int8_t>     map_data_;

    double  running_avg_;
    int     running_avg_ticks_;
    int     sampling_;

    double padding_;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "roimap");
    ros::NodeHandle n("~");

    ROIMapNode roimap(n);

    ros::spin();

    return 0;
}
