/**
 * @brief Only for testing some things.
 */
#include <cstdio>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include "mapprocessor.h"
#include <opencv2/highgui/highgui.hpp>

#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node_handle;
    /*
    MapProcessor map_processor;

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

    map_processor.setMap(map);
    ROS_INFO("Circle Traversability: %d", map_processor.checkTraversabilityOfCircle(Vector2i(50,20), 15));

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


    typedef vector<uchar> vc;
    vc foo(1000);
    for (size_t i = 0;  i< foo.size(); ++i) {
        foo[i] = (i%2 ? 0 : 100);
    }

    vc bar(1000);

    ros::Time start = ros::Time::now();

    for (int i = 0; i < 100000; ++i) {
        for (size_t j = 0; j < foo.size(); ++j) {
            bar[i] = (foo[i] == 0) ? 255 : 0;
        }
    }

    ros::Time end = ros::Time::now();

    for (vc::iterator it = bar.begin(); it != bar.end(); ++it) {
        cout << (int)*it << ", ";
    }
    cout << endl << endl;


    cout << "Duration: " << (end-start) << endl;

    ros::spin();

    return 0;
}

