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
    publish_normalized_   = node_handle_.advertise<sensor_msgs::LaserScan>("scan/flattend", 100);
    publish_differential_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan/differential", 100);
    publish_path_points_  = node_handle_.advertise<traversable_path::LaserScanClassification>("scan/traversable", 100);

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
    traversable_path::LaserScanClassification classification = detectObstacles(diff, msg->intensities);

    // publish modified message
    publish_path_points_.publish(classification);
    publish_normalized_.publish(msg);
    publish_differential_.publish(diff);
}

bool DisplayLaserData::calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Use current laser data for calibration.");

    // fetch one laser scan message, which will be used for calibration
    sensor_msgs::LaserScanConstPtr scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");

    this->plane_ranges_ = scan->ranges;
    this->is_calibrated_ = true;

    // store range-vector
    VectorSaver<float> vs(range_calibration_file_);
    vs.store(scan->ranges);

    return true;
}

vector<float> DisplayLaserData::smooth(std::vector<float> data, const unsigned int num_values)
{
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

traversable_path::LaserScanClassification DisplayLaserData::detectObstacles(sensor_msgs::LaserScan data,
                                                                            std::vector<float> &out)
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


    // Look at segemnts
    const unsigned int SEGMENT_SIZE = 10; //TODO unterteilung in segmente weglassen?
    const float RANGE_LIMIT = 0.005;
    const float INTENSITY_LIMIT = 13.0;

    // number of values
    const unsigned int LENGTH = data.ranges.size();
    const unsigned int NUM_SEGMENTS = ceil(LENGTH / SEGMENT_SIZE);
    
    static boost::circular_buffer< vector<PointClassification> > scan_buffer(3); //TODO class member instead io static variable

    vector<PointClassification> classification;

    for (unsigned int i = 0; i < LENGTH; i+=SEGMENT_SIZE) {
        float sum_range = 0;
        float sum_intensity = 0;
        PointClassification seg_class;

        for (unsigned int j = i; j < i+SEGMENT_SIZE && j < LENGTH; ++j) {
            sum_range += abs(data.ranges[i]);
            sum_intensity += abs(data.intensities[i]);
        }

        seg_class.traversable_by_range     = (    sum_range / SEGMENT_SIZE < RANGE_LIMIT    );
        seg_class.traversable_by_intensity = (sum_intensity / SEGMENT_SIZE < INTENSITY_LIMIT);

        classification.push_back(seg_class);
    }
    scan_buffer.push_back(classification);


    // points will only be marked as traversable if they are traversable in more than the half of the scans in the
    // scan buffer.
    int num_untraversable_due_to_intensity = 0;
    vector<PointClassification> segments( NUM_SEGMENTS );
    unsigned int store_size = scan_buffer.size();

    for (unsigned int i = 0; i < NUM_SEGMENTS; ++i) {
        int sum_traversable_range     = 0;
        int sum_traversable_intensity = 0;

        for (unsigned int j = 0; j < store_size; ++j) {
            if (scan_buffer[j][i].traversable_by_range)
                ++sum_traversable_range;

            if (scan_buffer[j][i].traversable_by_intensity)
                ++sum_traversable_intensity;
        }

        segments[i].traversable_by_range = (sum_traversable_range > store_size/2.0);
        segments[i].traversable_by_intensity = (sum_traversable_intensity > store_size/2.0);

        if (!segments[i].traversable_by_intensity) {
            ++num_untraversable_due_to_intensity;
        }
    }




    // remove single intensity-peaks
    const int MIN_SPACE_AROUND_INTESITY_PEAK = 2;
    const int MAX_INTENSITY_PEAK_SIZE = 2;
    const int STATE_BEFORE_PEAK = 0;
    const int STATE_PEAK = 1;
    const int STATE_AFTER_PEAK = 2;
    int intensity_state = STATE_BEFORE_PEAK;
    int intensity_peak_start = 0;
    int intensity_peak_size  = 0;
    int traversable_counter = 0;

    for (size_t i = 0; i < segments.size(); ++i) {
        //ROS_DEBUG("R:%d, I:%d, State: %d", result[i].traversable_by_range, result[i].traversable_by_intensity, intensity_state);

        switch (intensity_state) {
        case STATE_BEFORE_PEAK:
            if (segments[i].traversable_by_range && segments[i].traversable_by_intensity) {
                ++traversable_counter;
            } else {
                if (segments[i].traversable_by_range && traversable_counter >= MIN_SPACE_AROUND_INTESITY_PEAK) {
                    intensity_state = STATE_PEAK;
                    intensity_peak_start = i;
                    intensity_peak_size = 1;
                }

                traversable_counter = 0;
            }

            break;

        case STATE_PEAK:
            if (segments[i].traversable_by_range && segments[i].traversable_by_intensity) {
                intensity_state = STATE_AFTER_PEAK;
                ++traversable_counter;
            } else if (segments[i].traversable_by_range) { // only intensity
                if (++intensity_peak_size > MAX_INTENSITY_PEAK_SIZE) {
                    intensity_state = STATE_BEFORE_PEAK;
                }
            } else {
                intensity_state = STATE_BEFORE_PEAK;
            }

            break;

        case STATE_AFTER_PEAK:
            if (segments[i].traversable_by_range && segments[i].traversable_by_intensity) {
                if (++traversable_counter >= MIN_SPACE_AROUND_INTESITY_PEAK) {
                    ROS_DEBUG("Remove intensity peak");
                    for (int j = intensity_peak_start; j < intensity_peak_start+intensity_peak_size; ++j) {
                        segments[j].traversable_by_intensity = true;
                    }
                    intensity_state = STATE_BEFORE_PEAK;
                }
            } else {
                intensity_state = STATE_BEFORE_PEAK;
            }

            break;
        }
    }


    // drop traversable segments, that are too narrow for the robot
    //const float MIN_TRAVERSABLE_WIDTH = 0.7; // 70cm
    // the middle of the robot is approximatly in the middle of the scan data...
    //int mid = NUM_SEGMENTS/2;
    //FIXME simple for the beginning...
    const int MIN_TRAVERSABLE_SEGMENTS = 5;
    traversable_counter = 0;
    for (vector<PointClassification>::iterator seg_it = segments.begin(); seg_it != segments.end(); ++seg_it) {
        if (seg_it->traversable_by_range && seg_it->traversable_by_intensity) {
            ++traversable_counter;
        } else {
            //ROS_DEBUG("Width: untraversable. counter: %d", traversable_counter);
            if (traversable_counter > 0 && traversable_counter < MIN_TRAVERSABLE_SEGMENTS) {
                //ROS_DEBUG("Width: Drop narrow path");
                for (int i = 0; i < traversable_counter; ++i) {
                    --seg_it;
                    seg_it->traversable_by_range = false; //TODO: nicht range, dafür extra feld?
                }
                seg_it += traversable_counter;
            }
            traversable_counter = 0;
        }
    }

    // paint
    visualizer_.paintPath(classification);

    // only use intensity, if not more than 60% of the segments are untraversable due to intensity.
    bool use_intensity = true; //(float)num_untraversable_due_to_intensity / NUM_SEGMENTS < 0.60;

    traversable_path::LaserScanClassification result;
    result.traversable.resize(LENGTH);

    // missuse intensity of out as obstacle indicator... just for testing, I promise! :P
    for (unsigned int i = 0; i < LENGTH; ++i) {
        result.traversable[i] = segments[i/SEGMENT_SIZE].isTraversable();

        out[i] = 0.0;
        if (!segments[i/SEGMENT_SIZE].traversable_by_range)
            out[i] += 3000.0;
        if (use_intensity && !segments[i/SEGMENT_SIZE].traversable_by_intensity)
            out[i] += 5000.0;
    }

    return result;
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
