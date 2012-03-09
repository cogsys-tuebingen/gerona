#include "display_laser_data.h"
#include <stdio.h>
#include <algorithm>
#include <math.h>

using namespace std;

DisplayLaserData::DisplayLaserData() :
        isCalibrated(false)
{
    pub = n.advertise<sensor_msgs::LaserScan>("scan/flattend", 1000);

    // subscribe laser scanner
    sub = n.subscribe("scan", 1000, &DisplayLaserData::printLaserData, this);

    // register calibration service
    service = n.advertiseService("calibrate_plane", &DisplayLaserData::calibrate, this);
}

void DisplayLaserData::printLaserData(const sensor_msgs::LaserScanPtr &msg)
{
    // if laser is calibrated, subtract plane values
    if (this->isCalibrated) {
        for (uint i=0; i < msg->ranges.size(); ++i) {
            msg->ranges[i] = this->planeRanges[i];
        }
    }

    // mirror laser data, to make it fit with the camera image
//    reverse(msg->ranges.begin(), msg->ranges.end());
//    reverse(msg->intensities.begin(), msg->intensities.end());

    // publish modified message
    pub.publish(msg);
}

bool DisplayLaserData::calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Use current laser data for calibration.");

    // fetch one laser scan message, which will be used for calibration
    sensor_msgs::LaserScan scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan").get();

    //*
    // calibration by math
    int mid = scan.ranges.size()/2;

    // simple local search for minimum
//    cout << "mid: " << mid << endl;
//    while (msg->ranges[mid] > msg->ranges[mid-10] || msg->ranges[mid] > msg->ranges[mid+10]) {
//
//    }
//    if (msg->ranges[mid] > msg->ranges[mid-10]) {
//        mid-= 10;
//    } else {
//        mid+=10;
//    }
//
//    cout << "min: " << mid << endl << "----------" << endl;

//    int step = 100;
//    float valMid = msg->ranges[mid];
//    float valLeft = msg->ranges[mid-step];
//    float valRight = msg->ranges[mid+step];

    // IDEE: interpoliere messwerte, statt s/cos - s zu verwenden?
    // Oder interolation nur für minimum-suche? da gibts evtl bessere verfahren?


    double s = scan.ranges[mid];
    for (uint i=0; i < scan.ranges.size(); ++i) {
        double angle = scan.angle_increment * (i - mid);

        //if (abs(angle) > M_PI/2) {
        //    // remove values with angle > 90 degree
        //    scan.ranges[i] = 0;
        //} else {
            double d = s/cos(angle) - s;
            scan.ranges[i] = d;
        //}

        cout << scan.ranges[i] << endl;
    }

    //*/


    this->planeRanges = scan.ranges;
    this->isCalibrated = true;

    return true;
}


//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_laser_data");

    DisplayLaserData dls;

    // main loop
    ros::spin();
    return 0;
}
