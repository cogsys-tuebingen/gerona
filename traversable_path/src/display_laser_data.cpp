#include "display_laser_data.h"
#include <stdio.h>
#include <algorithm>
#include <numeric>
#include <math.h>
#include "vectorsaver.h"

using namespace std;

DisplayLaserData::DisplayLaserData() :
        is_calibrated_(false)
{
    pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/flattend", 100);
    publish_smooth_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/smooth", 100);
    publish_differential_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/differential", 100);

    // subscribe laser scanner
    sub_ = node_handle_.subscribe("scan", 100, &DisplayLaserData::printLaserData, this);

    // register calibration service
    service_ = node_handle_.advertiseService("calibrate_plane", &DisplayLaserData::calibrate, this);


    // look for existing range calibration file
    VectorSaver<float> vs(RANGE_CALIBRATION_FILE);
    is_calibrated_ = vs.load(&plane_ranges_);
}

void DisplayLaserData::printLaserData(const sensor_msgs::LaserScanPtr &msg)
{
    //FIXME publish als LaserScan nur, weil dann leicht mit dem laserscan_viewer anzeigbar :)
    sensor_msgs::LaserScan diff = *msg;
    sensor_msgs::LaserScan smoothed = *msg;
    //diff.ranges.resize(msg->ranges.size());

    // if laser is calibrated, subtract plane values
    if (this->is_calibrated_) {
        for (unsigned int i=0; i < msg->ranges.size(); ++i) {
            msg->ranges[i] -= this->plane_ranges_[i];

            // mirror on x-axis
            msg->ranges[i] *= -1;
        }
    }

    // smooth ranges
    smoothed.ranges = smooth(msg->ranges);

    // differential
    diff.ranges[0] = 0;
    for (unsigned int i=1; i < smoothed.ranges.size(); ++i)
    {
        diff.ranges[i] = smoothed.ranges[i] - smoothed.ranges[i-1];
    }

    // smooth differential
    //smoothed.ranges = smooth(diff.ranges);

    // mirror laser data, to make it fit with the camera image
//    reverse(msg->ranges.begin(), msg->ranges.end());
//    reverse(msg->intensities.begin(), msg->intensities.end());


    // publish modified message
    pub_.publish(msg);
    publish_smooth_.publish(smoothed);
    publish_differential_.publish(diff);
}

bool DisplayLaserData::calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Use current laser data for calibration.");

    // fetch one laser scan message, which will be used for calibration
    sensor_msgs::LaserScanConstPtr scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");
    //sensor_msgs::LaserScan scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan").get();

    /*
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


    this->plane_ranges_ = scan->ranges;
    this->is_calibrated_ = true;

    // store range-vector
    VectorSaver<float> vs(RANGE_CALIBRATION_FILE);
    vs.store(scan->ranges);

    return true;
}


vector<float> DisplayLaserData::smooth(std::vector<float> data)
{
    const unsigned int num_values = 4;
    list<float> neighbourhood;
    unsigned int length = data.size();

    // push first values to neighbourhood
    for (unsigned int i = 0; i < num_values && i < length; ++i)
    {
        neighbourhood.push_front(data[i]);
    }

    // push next and drop last
    for (unsigned int i = 0; i < length-num_values; ++i)
    {
        neighbourhood.push_front(data[i+num_values]);

        if (i > num_values+1)
        {
            neighbourhood.pop_back();
        }

        data[i] = avg(neighbourhood);
    }

    // nothing more to push
    for (unsigned int i = length-num_values; i < data.size(); ++i)
    {
        neighbourhood.pop_back();
        data[i] = avg(neighbourhood);
    }

    return data;
}

float DisplayLaserData::avg(std::list<float> &xs)
{
    if (xs.empty())
        return 0;
    else
        return accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
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
