#include "display_laser_data.h"
#include <stdio.h>
#include <algorithm>

using namespace std;

DisplayLaserData::DisplayLaserData()
{
    pub = n.advertise<sensor_msgs::LaserScan>("manipulated_scan", 1000);

    // subscribe laser scanner
    sub = n.subscribe("scan", 1000, &DisplayLaserData::printLaserData, this);
}

void DisplayLaserData::printLaserData(const sensor_msgs::LaserScan::Ptr &msg)
{
    // mirror laser data, to make it fit with the camera image
    reverse(msg->ranges.begin(), msg->ranges.end());
    reverse(msg->intensities.begin(), msg->intensities.end());

    // publish modified message
    pub.publish(msg);
}

void DisplayLaserData::calibrate()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_laser_data");

    DisplayLaserData dls;

    // main loop
    ros::spin();
    return 0;
}
