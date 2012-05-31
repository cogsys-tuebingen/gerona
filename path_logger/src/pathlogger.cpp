/*
 * statistics.cpp
 *
 *  Created on: 31.05.2012
 *      Author: buck
 */

#include <visualization_msgs/Marker.h>

#include "pathlogger.h"
#include <opencv2/opencv.hpp>

PathLogger::PathLogger()
    : has_last_pos_(false)
{
}

PathLogger::~PathLogger()
{
    if(latest_map_ != NULL){
        unsigned w = latest_map_->info.width;
        unsigned h = latest_map_->info.height;


        cv::Mat img(w, h, CV_8UC3, cv::Scalar(255, 128, 128, 255));

        uchar * data = img.data;

        unsigned nc = img.channels();
        unsigned ws = img.cols * nc;

        unsigned i = 0;
        for(int y = h-1; y >= 0; --y){
            for(unsigned x = 0; x < w; ++x){
                int map = latest_map_->data.at(i++);
                bool free = (map == 0);
                bool obstacle = map > 5;

                uchar rgb = obstacle ? 0 : (free ? 255 : 127);

                data[y * ws + x * nc] = rgb;
                data[y * ws + x * nc+1] = rgb;
                data[y * ws + x * nc+2] = rgb;
            }
        }

        unsigned n = positions_.size();
        if(n >= 2){
            double r = latest_map_->info.resolution;

            double ox = -latest_map_->info.origin.position.x;
            double oy = -latest_map_->info.origin.position.y;

            cv::Scalar color(255, 128, 0, 255);

            for(unsigned i = 1; i < n; ++i){
                geometry_msgs::Point &last_pt = positions_[i-1];
                geometry_msgs::Point &pt = positions_[i];
                cv::Point2d from((last_pt.x + ox) / r, h - 1 - (last_pt.y + oy) / r);
                cv::Point2d to((pt.x + ox) / r, h - 1 - (pt.y + oy) / r);

                cv::line(img, from, to, color, 1, CV_AA);
            }

        }
        cv::imwrite("/tmp/path.jpg", img);
        ROS_WARN_STREAM("wrote the trajectory to /tmp/path.jpg");

        cv::namedWindow("debug", CV_WINDOW_KEEPRATIO);
        cv::imshow("debug", img);
        cv::waitKey(3000);
    }
}

void PathLogger::init()
{
    ros::NodeHandle n("~");

    sub_ = n.subscribe<nav_msgs::OccupancyGrid>
            ("/map", 1, boost::bind(&PathLogger::map_callback, this, _1));

    pub_ = n.advertise<visualization_msgs::Marker>
            ("path", 1, true);
}

void PathLogger::map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    ROS_INFO("got map");

    latest_map_.reset(new nav_msgs::OccupancyGrid);
    latest_map_->header = msg->header;
    latest_map_->info = msg->info;
    latest_map_->data = msg->data;
}

void PathLogger::tick()
{
    if(!listener.frameExists("/map")){
        ROS_ERROR_STREAM("/map doesn't exist.");
        return;
    }

    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/map", "/base_link",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    tf::Vector3 pos = transform.getOrigin();
    if(!has_last_pos_ || pos.distance(last_pos_) > 0.1) {
        last_pos_ = pos;
        has_last_pos_ = true;

        geometry_msgs::Point pt;
        pt.x = pos[0];
        pt.y = pos[1];
        pt.z = pos[2];

        positions_.push_back(pt);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.15;
        marker.color.a = 1.0;
        marker.color.r = 0.2;
        marker.color.g = 0.2;
        marker.color.b = 1.0;

        marker.points = positions_;

        pub_.publish( marker );
    }
}

int main(int argc, char ** argv){
    ros::init(argc,argv, "path_logger");
    ros::NodeHandle n("~");

    PathLogger logger;
    logger.init();

    ros::Rate rate(1);

    while(ros::ok()){
        logger.tick();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

