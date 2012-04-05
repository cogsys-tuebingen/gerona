#include "terrainclassifier.h"
#include <stdio.h>
#include <algorithm>
#include <numeric>
#include <math.h>
#include "vectorsaver.h"

using namespace std;

TerrainClassifier::TerrainClassifier() :
        is_calibrated_(false),
        scan_buffer(3)
{
    // private node handle for parameter access
    ros::NodeHandle private_node_handle("~");

    // advertise
    publish_normalized_   = node_handle_.advertise<sensor_msgs::LaserScan>("scan/flattend", 100);
    publish_path_points_  = node_handle_.advertise<traversable_path::LaserScanClassification>("scan/traversability", 100);

    // subscribe laser scanner
    subscribe_laser_scan_ = node_handle_.subscribe("scan", 100, &TerrainClassifier::classifyLaserScan, this);

    // register calibration service
    calibration_service_ = node_handle_.advertiseService("calibrate_plane", &TerrainClassifier::calibrate, this);


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

void TerrainClassifier::classifyLaserScan(const sensor_msgs::LaserScanPtr &msg)
{
    // with uncalibrated laser, classification will not work
    if (!this->is_calibrated_) {
        return;
    }

    if (msg->intensities.size() == 0) {
        ROS_ERROR("Need intensity values of the laser scanner. Please reconfigure the laser scanner.");
        return;
    }

    sensor_msgs::LaserScan smoothed = *msg;

    // subtract plane calibration values to normalize the scan data
    for (unsigned int i=0; i < msg->ranges.size(); ++i) {
        smoothed.ranges[i] = msg->ranges[i] - plane_ranges_[i];

        // mirror on x-axis
        smoothed.ranges[i] *= -1;
    }

    // smooth
    smoothed.ranges = smooth(smoothed.ranges, 6);
    smoothed.intensities = smooth(msg->intensities, 4);

    // find obstacles
    traversable_path::LaserScanClassification classification;
    vector<bool> traversable = detectObstacles(smoothed, msg->intensities);

    // get projection to carthesian frame
    sensor_msgs::PointCloud cloud;
    laser_projector_.projectLaser(*msg, cloud, laser_geometry::channel_option::Index);
    classification.points = cloud.points;

    // connect points and traversability-values
    for (vector<sensor_msgs::ChannelFloat32>::iterator channel_it = cloud.channels.begin();
            channel_it != cloud.channels.end();
            ++channel_it) {
        //cout << "Channel: " << channel_it->name << endl; // das tut
        //ROS_INFO("Channel: %s", channel_it->name);       // das stuertzt ab - warum?
        if (channel_it->name.compare("index") == 0) {
            unsigned int val_size = channel_it->values.size();
            classification.traversable.resize(val_size);
            for (unsigned int i = 0; i < val_size; ++i) {
                unsigned int index = channel_it->values[i];
                classification.traversable[i] = traversable[index];
            }
        }
    }

    // publish modified message
    publish_path_points_.publish(classification);
    smoothed.intensities = msg->intensities;
    publish_normalized_.publish(smoothed); //< FIXME this topic is only for debugging
}

bool TerrainClassifier::calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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

vector<float> TerrainClassifier::smooth(std::vector<float> data, const unsigned int num_values)
{
    //FIXME range check
    boost::circular_buffer<float> neighbourhood(2*num_values + 1);
    unsigned int length = data.size();

    // push first values to neighbourhood
    for (unsigned int i = 0; i < num_values && i < length; ++i)
    {
        neighbourhood.push_back(data[i]);
    }

    // push next
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

float TerrainClassifier::avg(boost::circular_buffer<float> &xs)
{
    if (xs.empty())
        return 0.0;
    else
        return accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
}

vector<bool> TerrainClassifier::detectObstacles(sensor_msgs::LaserScan data, std::vector<float> &out)
{
    // parameters
    const unsigned int SEGMENT_SIZE = 10;       //!< Points per segment. //TODO unterteilung in segmente weglassen?
    const float DIFF_RANGE_LIMIT = 0.005;       //!< Limit of the range differential.
    const float DIFF_INTENSITY_LIMIT = 13.0;    //!< Limit of the intensity differential.


    // number of values
    const unsigned int LENGTH = data.ranges.size(); //!< Length of the scan vector.
    const unsigned int NUM_SEGMENTS = ceil(LENGTH / SEGMENT_SIZE); //!< Number of segments.

    // differentials
    vector<float> diff_ranges(LENGTH);          //!< range differential
    vector<float> diff_intensities(LENGTH);     //!< intensity differential
    
    for (unsigned int i=0; i < LENGTH - 1; ++i) {
        diff_ranges[i] = data.ranges[i] - data.ranges[i+1];
        diff_intensities[i] = abs(data.intensities[i] - data.intensities[i+1]); //< BEWARE of the abs()!
    }
    diff_ranges[LENGTH-1] = diff_ranges[LENGTH-2];
    diff_intensities[LENGTH-1] = diff_intensities[LENGTH-2];

    // smooth differential of intensity
    diff_intensities = smooth(diff_intensities, 6);


    // classify the current scan
    vector<PointClassification> classification; //!< Classification for each segment in the current scan.
    for (unsigned int i = 0; i < LENGTH; i+=SEGMENT_SIZE) {
        float sum_range = 0;
        float sum_intensity = 0;
        PointClassification seg_class;

        for (unsigned int j = i; j < i+SEGMENT_SIZE && j < LENGTH; ++j) {
            sum_range += abs(diff_ranges[i]);
            sum_intensity += abs(diff_intensities[i]);
        }

        seg_class.traversable_by_range     = (    sum_range / SEGMENT_SIZE < DIFF_RANGE_LIMIT    );
        seg_class.traversable_by_intensity = (sum_intensity / SEGMENT_SIZE < DIFF_INTENSITY_LIMIT);
        
//        // check height (this is important to detect walls in front)
//        if (abs(data.ranges[i]) > 0.3) {
//            seg_class.traversable_by_range = false;
//        }

        classification.push_back(seg_class);
    }
    
    
    
    // push current scn to buffer
    scan_buffer.push_back(classification);

    // points will only be marked as traversable if they are traversable in more than the half of the scans in the
    // scan buffer.
    int num_untraversable_due_to_intensity = 0;
    vector<PointClassification> segments( NUM_SEGMENTS );   //!< Final classification of the segments.
    unsigned int scan_buffer_size = scan_buffer.size();

    for (unsigned int i = 0; i < NUM_SEGMENTS; ++i) {
        int sum_traversable_range     = 0;
        int sum_traversable_intensity = 0;

        for (unsigned int j = 0; j < scan_buffer_size; ++j) {
            if (scan_buffer[j][i].traversable_by_range)
                ++sum_traversable_range;

            if (scan_buffer[j][i].traversable_by_intensity)
                ++sum_traversable_intensity;
        }

        segments[i].traversable_by_range = (sum_traversable_range > scan_buffer_size/2.0);
        segments[i].traversable_by_intensity = (sum_traversable_intensity > scan_buffer_size/2.0);

        if (!segments[i].traversable_by_intensity) {
            ++num_untraversable_due_to_intensity;
        }
    }


    removeSingleIntensityPeaks(segments);


    // drop traversable segments, that are too narrow for the robot
    //const float MIN_TRAVERSABLE_WIDTH = 0.7; // 70cm
    // the middle of the robot is approximatly in the middle of the scan data...
    //int mid = NUM_SEGMENTS/2;
    //FIXME simple for the beginning...
    const int MIN_TRAVERSABLE_SEGMENTS = 5;
    int traversable_counter = 0;
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

    // paint path
    /** @todo extra node for visualization */
    //visualizer_.paintPath(segments);

    // only use intensity, if not more than 60% of the segments are untraversable due to intensity.
    // deaktivated
    bool use_intensity = true; //(float)num_untraversable_due_to_intensity / NUM_SEGMENTS < 0.60;

    vector<bool> result;
    result.resize(LENGTH);

    // convert segment vector to point vector (each point is mapped to a bool. True = traversable, false=untraversable)
    for (unsigned int i = 0; i < LENGTH; ++i) {
        result[i] = segments[i/SEGMENT_SIZE].isTraversable();

        // missuse out as obstacle indicator... just for testing, I promise! :P
        out[i] = 0.0;
        if (!segments[i/SEGMENT_SIZE].traversable_by_range)
            out[i] += 3000.0;
        if (use_intensity && !segments[i/SEGMENT_SIZE].traversable_by_intensity)
            out[i] += 5000.0;
    }



    //visualizer_.plot(data.ranges);

    return result;
}

void TerrainClassifier::removeSingleIntensityPeaks(std::vector<PointClassification> &segments)
{
    //! Minimum number of traversable segments around a intensity peak that allows to ignore this peak.
    const int MIN_SPACE_AROUND_FALSE_INTESITY_PEAK = 3;
    //! Maximum size of an intensity peak (in segments) that allowes to ignore this peak.
    const int MAX_FALSE_INTENSITY_PEAK_SIZE = 3;

    // remove single intensity-peaks (using a simple state machine)
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
                if (segments[i].traversable_by_range && traversable_counter >= MIN_SPACE_AROUND_FALSE_INTESITY_PEAK) {
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
                if (++intensity_peak_size > MAX_FALSE_INTENSITY_PEAK_SIZE) {
                    intensity_state = STATE_BEFORE_PEAK;
                }
            } else {
                intensity_state = STATE_BEFORE_PEAK;
            }

            break;

        case STATE_AFTER_PEAK:
            if (segments[i].traversable_by_range && segments[i].traversable_by_intensity) {
                if (++traversable_counter >= MIN_SPACE_AROUND_FALSE_INTESITY_PEAK) {
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
}


//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "classify_terrain");

    TerrainClassifier dls;

    // main loop
    ros::spin();
    return 0;
}
