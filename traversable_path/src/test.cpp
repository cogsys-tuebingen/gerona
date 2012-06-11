/**
 * @brief Only for testing some things.
 */
#include <cstdio>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include "mapprocessor.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

void printMap(const nav_msgs::OccupancyGrid &map) {
    for (size_t i = 0; i < map.data.size(); ++i) {
        cout << (map.data[i] == 0 ? 'F' : (map.data[i] == 100 ? 'O' : '?'));

        if ((i+1)%map.info.width == 0) {
            cout << endl;
        }
    }
}

void printImg(const cv::Mat1b &img) {
    for (int x = 0; x < img.cols; ++x) {
        for (int y = 0; y < img.rows; ++y) {
            switch (img[x][y]) {
            case 0:
                cout << 'O';
                break;
            case 255:
                cout << 'F';
            default:
                cout << '?';
            }
        }
        cout << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node_handle;

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

    return 0;
}

