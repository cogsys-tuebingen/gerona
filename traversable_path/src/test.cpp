/**
 * @brief Only for testing some things.
 */
#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node_handle;

    nav_msgs::OccupancyGrid map;

    map.info.resolution = 0.05;
    map.info.width  = 10;
    map.info.height = 5;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(map.info.width * map.info.height, 0);
    map.header.frame_id = "/map";

    int x = 3, y = 1;

    size_t index = y * map.info.width + x;

    map.data[index] = 100;

    ros::Publisher pub = node_handle.advertise<nav_msgs::OccupancyGrid>("/testmap", 0);

    ros::Rate rate(1);
    while(ros::ok()) {
        pub.publish(map);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

