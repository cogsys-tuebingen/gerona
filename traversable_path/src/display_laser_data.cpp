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
    // private node handle for parameter access
    ros::NodeHandle private_node_handle("~");

    // advertise
    publish_normalized_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/flattend", 100);
    publish_smooth_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/smooth", 100);
    publish_differential_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/differential", 100);

    // subscribe laser scanner
    subscribe_laser_scan_ = node_handle_.subscribe("scan", 100, &DisplayLaserData::printLaserData, this);

    // register calibration service
    service_ = node_handle_.advertiseService("calibrate_plane", &DisplayLaserData::calibrate, this);


    // load range calibration filename from parameter
    private_node_handle.param<string>("calibration_file", range_calibration_file_, DEFAULT_RANGE_CALIBRATION_FILE);
    if (range_calibration_file_.compare("default") == 0) {
        range_calibration_file_ = DEFAULT_RANGE_CALIBRATION_FILE;
    }
    ROS_INFO("Using calibration file %s", range_calibration_file_.c_str());

    // look for existing range calibration file
    VectorSaver<float> vs(range_calibration_file_);
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

    // smooth
    smoothed.ranges = smooth(msg->ranges, 6);
    smoothed.intensities = smooth(msg->intensities, 4);

    // differential
    diff.ranges[0] = 0;
    diff.intensities[0] = 0;
    for (unsigned int i=1; i < smoothed.ranges.size(); ++i)
    {
        diff.ranges[i] = smoothed.ranges[i] - smoothed.ranges[i-1];
        diff.intensities[i] = abs(smoothed.intensities[i] - smoothed.intensities[i-1]); //< BEWARE of the abs()!
    }

    // smooth differential
    diff.intensities = smooth(diff.intensities, 6);

    // find obstacles
    detectObstacles(diff, msg->intensities);

    // publish modified message
    publish_normalized_.publish(msg);
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
    VectorSaver<float> vs(range_calibration_file_);
    vs.store(scan->ranges);

    return true;
}


vector<float> DisplayLaserData::smooth(std::vector<float> data, const unsigned int num_values)
{
    //list<float> neighbourhood;
    boost::circular_buffer<float> neighbourhood(2*num_values + 1);
    unsigned int length = data.size();

    // push first values to neighbourhood
    for (unsigned int i = 0; i < num_values && i < length; ++i)
    {
        neighbourhood.push_back(data[i]);
    }

    // push next and drop last
    for (unsigned int i = 0; i < length-num_values; ++i)
    {
        neighbourhood.push_back(data[i+num_values]);
        data[i] = avg(neighbourhood);
    }

    // nothing more to push
    for (unsigned int i = length-num_values; i < data.size(); ++i)
    {
        neighbourhood.pop_front();
        data[i] = avg(neighbourhood);
    }

    return data;
}

float DisplayLaserData::avg(boost::circular_buffer<float> &xs)
{
    if (xs.empty())
        return 0.0;
    else
        return accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
}

void DisplayLaserData::detectObstacles(sensor_msgs::LaserScan data, std::vector<float> &out)
{
    /* look at each point seperate
    const float RANGE_LIMIT = 0.01;
    const float INTENSITY_LIMIT = 40.0;

    for (unsigned int i = 0; i < data.ranges.size(); ++i) {
        // missuse intensity of out as obstacle indicator... just for testing, I promise! :P
        out[i] = 0;

        if (abs(data.ranges[i]) > RANGE_LIMIT) {
            out[i] = 3000; // just some high value that will be visible in the laserscan viewer
        }

        //cout << abs(data.intensities[i]) << endl;
        if (abs(data.intensities[i]) > INTENSITY_LIMIT) {
            out[i] = 5000; // an other high value to distinct range an intensity
        }
    }
    //*/


    //* Look at segemnts
    const unsigned int SEGMENT_SIZE = 10;
    const float RANGE_LIMIT = 0.005;
    const float INTENSITY_LIMIT = 12.0;

    // number of values
    const unsigned int length = data.ranges.size();
    
    static boost::circular_buffer< vector<PointClassification> > store(3); //TODO class member instead io static variable

    vector<PointClassification> classification;
    //classification.resize(data.ranges.size(), 0);

    for (unsigned int i = 0; i < length; i+=SEGMENT_SIZE) {
        float sum_range = 0;
        float sum_intensity = 0;
        PointClassification seg_class;

        for (unsigned int j = i; j < i+SEGMENT_SIZE && j < length; ++j) {
            sum_range += abs(data.ranges[i]);
            sum_intensity += abs(data.intensities[i]);
        }

//        if (sum_range/SEGMENT_SIZE > RANGE_LIMIT) {
//            for (unsigned int j = i; j < i+SEGMENT_SIZE && j < data.ranges.size(); ++j)
//                classification[j] = 3000.0; // just some high value that will be visible in the laserscan viewer
//        }
        seg_class.traversable_by_range = (sum_range/SEGMENT_SIZE < RANGE_LIMIT);

//        if (sum_intensity/SEGMENT_SIZE > INTENSITY_LIMIT) {
//            for (unsigned int j = i; j < i+SEGMENT_SIZE && j < data.ranges.size(); ++j)
//                classification[j] += 5000.0; // an other high value to distinct range an intensity
//        }
        seg_class.traversable_by_intensity = (sum_intensity/SEGMENT_SIZE < INTENSITY_LIMIT);

        classification.push_back(seg_class);
    }
    store.push_back(classification);

//    out.assign(out.size(), 5000.0);
//    for (boost::circular_buffer< vector<PointClassification> >::iterator it = store.begin(); it != store.end(); ++it) {
//        for (unsigned int i = 0; i < it->size(); ++i) {
//            float val = 0.0;
//            if (it->at(i).traversable_by_range)
//                val += 3000.0;
//            if (it->at(i).traversable_by_intensity)
//                val += 5000.0;
//
//            out[i] = min(out[i], val);
//        }
//    }

    //TODO number of segments to variable
    vector<PointClassification> result( store[0].size() );
    unsigned int store_size = store.size();
    for (unsigned int i = 0; i < store[0].size(); ++i) {
        int sum_traversable_range     = 0;
        int sum_traversable_intensity = 0;

        for (unsigned int j = 0; j < store_size; ++j) {
            if (store[j][i].traversable_by_range)
                ++sum_traversable_range;

            if (store[j][i].traversable_by_intensity)
                ++sum_traversable_intensity;
        }

        result[i].traversable_by_range = (sum_traversable_range > store_size/2.0);
        result[i].traversable_by_intensity = (sum_traversable_intensity > store_size/2.0);
    }


    // missuse intensity of out as obstacle indicator... just for testing, I promise! :P
    for (unsigned int i = 0; i < length; ++i) {
        out[i] = 0.0;
        if (!result[i/SEGMENT_SIZE].traversable_by_range)
            out[i] += 3000.0;
        if (!result[i/SEGMENT_SIZE].traversable_by_intensity)
            out[i] += 5000.0;
    }


    //*/
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
