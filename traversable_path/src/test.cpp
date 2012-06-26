/**
 * @brief Only for testing some things.
 */
#include <cstdio>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include "mapprocessor.h"
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Empty.h>

using namespace std;
using namespace Eigen;

ros::Timer timer;

void timerCallback(const ros::TimerEvent &event)
{
    cout << "Timer!" << endl;
}

void foo(const std_msgs::EmptyConstPtr &msg)
{
    cout << "Start Timer" << endl;
    timer.start();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node_handle;

    /*
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.05;
    map.info.width  = 100;
    map.info.height = 50;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(map.info.width * map.info.height, 0);

    map.data[0] = 100;
    // x = 8, y = 3;
    size_t index = 3 * map.info.width + 8;
    map.data[index] = 100;


    ros::Publisher pub = node_handle.advertise<nav_msgs::OccupancyGrid>("testmap", 0);

    cv::namedWindow("testmap", CV_WINDOW_FREERATIO);

    cv::Mat1b img;
    MapProcessor::mapToImage(map, &img);


    cout << "map: " << map.info.width << "x" << map.info.height << endl;
    cout << "img: " << img.cols << "x" << img.rows << endl;

    cv::circle(img, cv::Point(80, 30), 15, cv::Scalar(0));
    cv::line(img, cv::Point(80, 30), cv::Point(50, 0), cv::Scalar(0));

    MapProcessor::imageToMap(img, &map);


    ros::Rate rate(1);
    while(ros::ok()) {
        ros::spinOnce();

        cv::imshow("testmap", img);
        cv::waitKey(3);

        pub.publish(map);
        rate.sleep();
    }
    */

    ros::Subscriber sub = node_handle.subscribe("foo", 0, &foo);
    timer = node_handle.createTimer(ros::Duration(5), timerCallback, true, false);

    ros::spin();

    return 0;
}

