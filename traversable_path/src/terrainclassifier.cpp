#include "terrainclassifier.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

#include <ros/package.h>

#include <ramaxx_msgs/PTZ.h>
#include "calibrationdatastorage.h"

using namespace std;
using namespace traversable_path;


const std::string TerrainClassifier::DEFAULT_RANGE_CALIBRATION_FILE = ros::package::getPath(ROS_PACKAGE_NAME)
                                                             + std::string("/rangecalibration.yaml");

TerrainClassifier::TerrainClassifier() :
        is_calibrated_(false),
        scan_buffer_(3),
        save_next_scan_(false)
{
    // private node handle for parameter access
    ros::NodeHandle private_node_handle("~");

    // advertise
    publish_normalized_           = node_handle_.advertise<sensor_msgs::LaserScan>("scan/flattend", 100);
    publish_normalized_diff_       = node_handle_.advertise<sensor_msgs::LaserScan>("scan/normalized_diff", 100);
    publish_classification_cloud_[0] = node_handle_.advertise<PointCloudXYZRGBT >("path_classification_cloud0", 10);
    publish_classification_cloud_[1] = node_handle_.advertise<PointCloudXYZRGBT >("path_classification_cloud1", 10);
    publish_classification_cloud_[2] = node_handle_.advertise<PointCloudXYZRGBT >("path_classification_cloud2", 10);
    publish_classification_cloud_[3] = node_handle_.advertise<PointCloudXYZRGBT >("path_classification_cloud3", 10);
    publish_map_                  = node_handle_.advertise<nav_msgs::OccupancyGrid>("traversability_map", 1);

    // publish map in constant intervals of 100ms
    map_publish_timer_ = node_handle_.createTimer(ros::Duration(0.1), &TerrainClassifier::publishMap, this);

    /// Use message filters for the scan messages as this is recommended in the wiki for use with tf:
    /// http://www.ros.org/wiki/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData#Converting_a_Laser_Scan_to_a_Point_Cloud

    // subscribe tilted laser scanner
    // The ibeo LUX has 4 layers and there is a topic for each layer -> four subscribers.
    for (int layer = 0; layer < 4; ++layer) {
        ostringstream topic;
        topic << "/sick_ldmrs/scan" << layer;

        subscriber_laser_scan_[layer].subscribe(node_handle_, topic.str(), 100);
        message_filter_tilted_scan_[layer] = new tf::MessageFilter<sensor_msgs::LaserScan>
                (subscriber_laser_scan_[layer], tf_listener_, "/map", 100);
        message_filter_tilted_scan_[layer]->registerCallback( boost::bind(&TerrainClassifier::classifyLaserScan, this, _1, layer) );
    }


    // subscribe front scanner
    subscriber_front_scan_.subscribe(node_handle_, "scan", 10);
    message_filter_front_scan_ = new tf::MessageFilter<sensor_msgs::LaserScan>
            (subscriber_front_scan_, tf_listener_, "/map", 10);
    message_filter_front_scan_->registerCallback(&TerrainClassifier::frontScanCallback, this);


    // register calibration service
    calibration_service_ = node_handle_.advertiseService("calibrate_plane", &TerrainClassifier::calibrate, this);


    // load range calibration filename from parameter
    private_node_handle.param<string>("calibration_file", range_calibration_file_, DEFAULT_RANGE_CALIBRATION_FILE);
    if (range_calibration_file_.compare("default") == 0) {
        range_calibration_file_ = DEFAULT_RANGE_CALIBRATION_FILE;
    }
    ROS_INFO("Using calibration file %s", range_calibration_file_.c_str());

    // look for existing range calibration file
    CalibrationDataStorage cds(range_calibration_file_);
    is_calibrated_ = cds.load(&plane_ranges_);
    feature_calculator_.setCalibrationScan(plane_ranges_);


    // initialize map
    map_.info.resolution = 0.05; // 5cm per cell
    map_.info.width  = 200;
    map_.info.height = 200;
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    map_.data.resize(map_.info.width * map_.info.height, MAP_DEFAULT_VALUE );


    // register reconfigure callback (which will also initialize config_ with the default values)
    reconfig_server_.setCallback(boost::bind(&TerrainClassifier::dynamicReconfigureCallback, this, _1, _2));
}

TerrainClassifier::~TerrainClassifier()
{
    delete[] message_filter_tilted_scan_;
    delete message_filter_front_scan_;
}

void TerrainClassifier::dynamicReconfigureCallback(Config &config, uint32_t level)
{
    config_ = config;

    feature_calculator_.setVarianceWindowSize(config.variance_window_size);

    ROS_DEBUG("Reconfigure TerrainClassifier.");
}

void TerrainClassifier::classifyLaserScan(const sensor_msgs::LaserScanConstPtr &msg, uint layer)
{
//    ros::Time start_time = ros::Time::now();

    // with uncalibrated laser, classification will not work
    if (!this->is_calibrated_) {
        return;
    }

    if (msg->intensities.size() == 0) {
        ROS_ERROR("Need intensity values of the laser scanner. Please reconfigure the laser scanner.");
        return;
    }

    if (plane_ranges_.size() <= layer) {
        ROS_ERROR("No calibration data of layer %d. Please calibrate the laser scanner.", layer);
        return;
    }

    if (msg->ranges.size() != plane_ranges_[layer].size()) {
        ROS_ERROR("Size of calibration data of layer %d does not fit. Please calibrate the laser scanner.", layer);
        return;
    }


    vector<PointClassification> traversable;

    feature_calculator_.setScan(*msg, layer);

    // find obstacles
    vector<float> debug_classification_as_intensity(msg->intensities.size());
    traversable = detectObstacles(layer, debug_classification_as_intensity);

    // get projection to carthesian frame
    PointCloudXYZRGBT pcl_cloud;
    laserScanToCloud(msg, traversable, &pcl_cloud); /** \todo use smoothed instead of msg? */

    classifyPointCloud(&pcl_cloud);

    updateMap(pcl_cloud);

    if (save_next_scan_) {
        save_next_scan_ = false;
        sensor_msgs::LaserScan normalized_scan = feature_calculator_.getPreprocessedScan();
        scanToFile("/localhome/widmaier/scan_raw.dat", *msg);
        scanToFile("/localhome/widmaier/scan_normalized.dat", normalized_scan);
        normalized_scan.intensities = debug_classification_as_intensity;
        scanToFile("/localhome/widmaier/scan_classification.dat", normalized_scan);
    }

    // publish modified message
    publish_classification_cloud_[layer].publish(pcl_cloud);
    //ROS_DEBUG("Published %zu traversability points", pcl_cloud.points.size());
    /** @todo This topic is only for debugging. Remove in later versions */
    if ( (publish_normalized_.getNumSubscribers() > 0) && (layer == 1) ) {
        sensor_msgs::LaserScan normalized_scan = feature_calculator_.getPreprocessedScan();
        normalized_scan.intensities = debug_classification_as_intensity;
        publish_normalized_.publish(normalized_scan);
    }



//    ros::Time end_time = ros::Time::now();
//    ros::Duration running_duration = end_time - start_time;
//    ROS_DEBUG("classify scan duration: %fs", running_duration.toSec());
}

void TerrainClassifier::frontScanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    // transform to point cloud
    sensor_msgs::PointCloud cloud_msg;
    try
    {
       laser_projector_.transformLaserScanToPointCloud("/map", *msg, cloud_msg, tf_listener_,
                                                       -1.0, laser_geometry::channel_option::None);
    }
    catch (tf::TransformException& e)
    {
        ROS_WARN_THROTTLE_NAMED(1, "tf", "Unable to transform front laser scan. tf says: %s", e.what());
        return;
    }


    // assign points and traversability-data to the pcl point cloud
    PointCloudXYZRGBT cloud;
    cloud.header = cloud_msg.header;
    cloud.reserve(cloud_msg.points.size());
    for (vector<geometry_msgs::Point32>::const_iterator it = cloud_msg.points.begin(); it != cloud_msg.points.end(); ++it) {
        PointXYZRGBT point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        // all points of the horizontal scanner are not traversable
        point.traversable = false;

        cloud.push_back(point);
    }


    updateMap(cloud);
}

void TerrainClassifier::laserScanToCloud(const sensor_msgs::LaserScanConstPtr &scan,
                                         const vector<PointClassification> &traversable, PointCloudXYZRGBT *cloud)
{
    // get projection to carthesian frame
    sensor_msgs::PointCloud cloud_msg;
    //sensor_msgs::ChannelFloat32 channel_index;

    try {
        tf_listener_.waitForTransform("/odom", scan->header.frame_id, ros::Time(0), ros::Duration(0.01));
        /** \todo use /map instead of /odom? */
        laser_projector_.transformLaserScanToPointCloud("/odom", *scan, cloud_msg, tf_listener_, -1.0,
                                                        laser_geometry::channel_option::Index);
    }
    catch (tf::TransformException e) {
        ROS_WARN_THROTTLE_NAMED(1, "tf", "Unable to transform laser scan. tf says: %s", e.what());
        return;
    }

    // get index channel
    vector<sensor_msgs::ChannelFloat32>::iterator channel_it;
    for (channel_it = cloud_msg.channels.begin(); channel_it != cloud_msg.channels.end(); ++channel_it) {
        if (channel_it->name.compare("index") == 0) {
            //channel_index = *channel_it;
            break;
        }
    }

    // assign points and traversability-data to the pcl point cloud
    cloud->header = cloud_msg.header;
    cloud->reserve(cloud_msg.points.size());
    for (size_t i = 0; i < cloud_msg.points.size(); ++i) {
        //size_t index = channel_index.values[i];
        size_t index = channel_it->values[i];

        PointXYZRGBT point;
        point.x = cloud_msg.points[index].x;
        point.y = cloud_msg.points[index].y;
        point.z = cloud_msg.points[index].z;

        point.traversable = traversable[index].isTraversable();

        // color (for visualization)
        if (point.traversable) {
            // traversable. green to yellow depending on obstacle_value.
            point.r = (float) traversable[i].obstacle_value() / PointClassification::OBSTACLE_VALUE_LIMIT * 255;
            point.g = 255;
            point.b = 0;
        } else {
            if (traversable[index].classification().test(PointClassification::FLAG_VARIANCE_OVER_LIMIT)) {
                // untraversable -> red
                point.r = 255; point.g = 0; point.b = 0;
            } else {
                // for evaluation: untraversable points with *no* variance over limit are blue
                point.r = 0; point.g = 0; point.b = 255;
            }
        }

        cloud->push_back(point);
    }
}

bool TerrainClassifier::scanToFile(string filename, const sensor_msgs::LaserScan &scan)
{
    // write data to file, truncate file before writing
    std::ofstream out_file(filename.c_str(), std::ios::out | std::ios::trunc);
    if (!out_file.is_open()) {
        ROS_ERROR("scanToFile could not open file");
        return false;
    }

    out_file << "#angle\trange\tintensity\n";

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float angle = scan.angle_min + i*scan.angle_increment;
        out_file << angle << "\t" << scan.ranges[i] << "\t" << scan.intensities[i] << "\n";
    }

    if (!out_file.good()) {
        ROS_ERROR("scanToFile: Failure when writing data to file.");
        return false;
    }
    return true;
}

bool TerrainClassifier::calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Use current laser data for calibration.");

    // fetch one laser scan message, which will be used for calibration
    vector<vector<float> > scans(4);
    scans[0] = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/sick_ldmrs/scan0")->ranges;
    scans[1] = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/sick_ldmrs/scan1")->ranges;
    scans[2] = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/sick_ldmrs/scan2")->ranges;
    scans[3] = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/sick_ldmrs/scan3")->ranges;


    this->plane_ranges_  = scans;
    this->is_calibrated_ = true;
    feature_calculator_.setCalibrationScan(plane_ranges_);

    // store range-vector
    CalibrationDataStorage cds(range_calibration_file_);
    return cds.store(scans);
}

vector<PointClassification> TerrainClassifier::detectObstacles(uint layer, std::vector<float> &out)
{
    vector<float> diff_ranges = feature_calculator_.rangeDerivative();
    vector<float> diff_intensities = feature_calculator_.intensityDerivative();
    vector<float> range_variance = feature_calculator_.rangeVariance();

    const size_t LENGTH = diff_ranges.size(); //!< Length of the scan vector.

    //////
    if ( (publish_normalized_diff_.getNumSubscribers() > 0) && (layer == 1) ) {
        sensor_msgs::LaserScan scan_diff;
        scan_diff = feature_calculator_.getPreprocessedScan();
        //scan_diff.ranges = diff_ranges;
        //scan_diff.intensities = diff_intensities;
        scan_diff.ranges = range_variance;
        scan_diff.intensities = vector<float>(0);
        publish_normalized_diff_.publish(scan_diff);
    }
    //////

    vector<PointClassification> scan_classification(LENGTH);

    // classification
    for (size_t i = 0; i < LENGTH; ++i) {
        if (abs(diff_ranges[i]) > config_.diff_range_limit) {
            scan_classification[i].setFlag(PointClassification::FLAG_DIFF_RANGE_OVER_LIMIT);
        }

        if (range_variance[i] > config_.variance_threshold) {
            scan_classification[i].setFlag(PointClassification::FLAG_VARIANCE_OVER_LIMIT);
        }

        if (abs(diff_intensities[i]) > config_.diff_intensity_limit) {
            scan_classification[i].setFlag(PointClassification::FLAG_DIFF_INTENSITY_OVER_LIMIT);
        }

        /**
         * missuse out as obstacle indicator... just for testing, I promise! :P
         * @todo remove that in later versions!
         */
        out[i] = scan_classification[i].obstacle_value() * 20;
    }


    checkPointNeighbourhood(&scan_classification);

    // push current scan to buffer (push_front so the current scan has index 0)
    scan_buffer_.push_front(scan_classification);

    // points will only be marked as traversable if they are traversable in more than the half of the scans in the
    // scan buffer.
    const unsigned int scan_buffer_size = scan_buffer_.size();

    for (unsigned int i = 0; i < LENGTH; ++i) {
        unsigned int sum_traversable = 0;

        for (unsigned int j = 0; j < scan_buffer_size; ++j) {
            if (scan_buffer_[j][i].isTraversable())
                ++sum_traversable;
        }

        if (sum_traversable < scan_buffer_size/2) {
            scan_classification[i].setFlag(PointClassification::FLAG_UNTRAVERSABLE_IN_PAST_SCANS);
        }
    }


    /**
     * missuse out as obstacle indicator... just for testing, I promise! :P
     * @todo remove that in later versions!
     */
    for (unsigned int i = 0; i < LENGTH; ++i) {
        if (!scan_classification[i].isTraversable())
            out[i] = 100.0;
        else
            out[i] = 0.0;
    }

    return scan_classification;
}


void TerrainClassifier::checkPointNeighbourhood(vector<PointClassification> *scan_classification) const
{
    /*
     * Check neighbourhood of the points. This is done by buffering NEIGHBOURHOOD_RANGE points before and after the
     * current point and iterate over this buffer.
     * If a feature-flag is set in more than the half of this points, an according neighbourhood-flag is set.
     */
    //! Range of the neighbourhood (Number of points before and after the current point, NOT total number of points).
    const short NEIGHBOURHOOD_RANGE = 30;
    //! Length of the scan vector.
    const size_t LENGTH = scan_classification->size();
    boost::circular_buffer<PointClassification> neighbourhood(NEIGHBOURHOOD_RANGE*2+1);

    // insert first NEIGHBOURHOOD_RANGE elements of the scan classification to the neighbouthood.
    neighbourhood.insert(neighbourhood.begin(), scan_classification->begin(),
                         scan_classification->begin() + NEIGHBOURHOOD_RANGE);

    for (size_t i = 0; i < LENGTH; ++i) {
        // maintain neighbourhood
        if (i < LENGTH - NEIGHBOURHOOD_RANGE) {
            neighbourhood.push_back((*scan_classification)[i+NEIGHBOURHOOD_RANGE]);
        } else {
            neighbourhood.pop_front();
        }

        // Counters for the features. Will be incremented for each point with this feature and decremented for each
        // point without (so counter > 0 means, the feature is detected in more than the half of the points).
        short diff_intensity_neighbours = 0;
        short diff_range_neighbours = 0;
        // iterate over neighbourhood
        boost::circular_buffer<PointClassification>::iterator neighbour_it;
        for (neighbour_it = neighbourhood.begin(); neighbour_it != neighbourhood.end(); ++neighbour_it) {
            // count points with DIFF_INTENSITY_OVER_LIMIT
            if (neighbour_it->classification().test(PointClassification::FLAG_DIFF_INTENSITY_OVER_LIMIT)) {
                ++diff_intensity_neighbours;
            } else {
                --diff_intensity_neighbours;
            }

            // count points with DIFF_RANGE_OVER_LIMIT
            if (neighbour_it->classification().test(PointClassification::FLAG_DIFF_RANGE_OVER_LIMIT)) {
                ++diff_range_neighbours;
            } else {
                --diff_range_neighbours;
            }
        }

        // Check counters and set according flags.
        if (diff_intensity_neighbours > 0) {
            (*scan_classification)[i].setFlag(PointClassification::FLAG_DIFF_INTENSITY_NEIGHBOUR);
        }
        if (diff_range_neighbours > 0) {
            (*scan_classification)[i].setFlag(PointClassification::FLAG_DIFF_RANGE_NEIGHBOUR);
        }
    }
}


void TerrainClassifier::classifyPointCloud(PointCloudXYZRGBT *cloud) const
{
    /* **** Check traversable segments **** */
    size_t index_start = 0;       //!< Index of the first point of a traversable segment.
    bool on_trav_segment = false; //!< True if currently iterating in a traversable segment, otherwise false.

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (on_trav_segment && !cloud->points[i].traversable) {
            // end traversable segment
            on_trav_segment = false;

            //ROS_DEBUG("Check traversable segment (index %zu to %zu)", index_start, i-1);
            checkTraversableSegment(cloud->points.begin()+index_start, cloud->points.begin()+i);
        }
        else if (!on_trav_segment && cloud->points[i].traversable) {
            // begin new traversable segment
            on_trav_segment = true;
            index_start = i;
        }
        // else continue;
    }

    // if the last point is part of a traversable segment, the loop above will not check this segment any more.
    if (on_trav_segment) {
        ROS_DEBUG("Check traversable segment (index %zu to %zu)", index_start, cloud->points.size()-1);
        checkTraversableSegment(cloud->points.begin()+index_start, cloud->points.end());
    }
}

void TerrainClassifier::checkTraversableSegment(PointCloudXYZRGBT::iterator begin,
                                                PointCloudXYZRGBT::iterator end) const
{
    PointXYZRGBT point_start = *begin;
    PointXYZRGBT point_end = *(end-1);

    /* *********** check path width ************* */
    // width check is bad when working on a map, so comment it here.
//    double distance = sqrt( pow(point_start.x - point_end.x, 2) +
//                            pow(point_start.y - point_end.y, 2) +
//                            pow(point_start.z - point_end.z, 2) );

//    if (distance < config_.min_path_width) {
//        ROS_DEBUG("Drop too narrow path (distance: %.2fm)", distance);
//        goto untraversable;
//    }


    /* *********** check slope ************* */
    {
        const double b = fabs(point_start.z - point_end.z);
        const double a = sqrt( pow(point_start.x - point_end.x, 2) + pow(point_start.y - point_end.y, 2));
        double angle_of_slope = atan2(b,a);

        if (angle_of_slope > 0.2) { // To drop "paths" on walls, the limit angle depends on the laser tilt
            ROS_DEBUG("Drop too steep path (angle: %.2f, distance: %.2f)", angle_of_slope, a);
            goto untraversable;
        }
    }

    return;

    // sorry for the goto. I found no better solution to avoid copy pasting of the following code.
    untraversable:
    // make this segment untraversable
    for (PointCloudXYZRGBT::iterator cloud_it = begin; cloud_it != end; ++cloud_it) {
        cloud_it->setTraversability(false);
    }
}


void TerrainClassifier::updateMap(PointCloudXYZRGBT cloud)
{
    for (PointCloudXYZRGBT::iterator point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
        int x, y;
        x = (point_it->x - map_.info.origin.position.x) / map_.info.resolution;
        y = (point_it->y - map_.info.origin.position.y) / map_.info.resolution;

        // (int)-casts to supress comparison warning (no danger of overflow here)
        if (x < (int)map_.info.width && y < (int)map_.info.height && x >= 0 && y >= 0) {
            size_t index = y * map_.info.width + x;
            // 0 = traversable, 100 = untraversable
            map_.data[index] += point_it->traversable ? -10 : 10;

            // check range
            if (map_.data[index] < 0) {
                map_.data[index] = 0;
            } else if(map_.data[index] > 100) {
                map_.data[index] = 100;
            }
        } else {
            //ROS_WARN("Out of Map. (row,col) = (%d, %d)", col, row);
        }
    }
}

void TerrainClassifier::publishMap(const ros::TimerEvent &foo)
{
    // move map if necessary
    moveMap();

    nav_msgs::OccupancyGrid final_map = map_;
    final_map.header.stamp = ros::Time::now();

    // "binarize" map (only 0 and 100, no uncertain values between)
    for (vector<int8_t>::iterator map_it = final_map.data.begin(); map_it != final_map.data.end(); ++map_it) {
        *map_it = *map_it < 50 ? 0 : 100;
    }

    // remove noise
    map_processor_.process(&final_map);

    publish_map_.publish(final_map);
}

void TerrainClassifier::moveMap()
{
    //! The minimum distance, the robot must have moved, to recenter the map to the robot.
    const float MIN_ROBOT_MOVEMENT_DISTANCE = 2.0;

    // get robot position
    geometry_msgs::PointStamped base_link_position, map_origin;
    base_link_position.header.frame_id = "/base_link";
    // x,y,z = 0 is already initialization default
    //base_link_position.point.x = 0.0;
    //base_link_position.point.y = 0.0;
    //base_link_position.point.z = 0.0;

    try {
        tf_listener_.transformPoint("/map", base_link_position, map_origin);
    }
    catch (tf::TransformException e) {
        ROS_WARN_THROTTLE(1, "Unable to transform robot position. tf says: %s", e.what());
        return;
    }
    // set origin so that the robot is in the center of the map
    map_origin.point.x -= (signed int) map_.info.width  * map_.info.resolution / 2;
    map_origin.point.y -= (signed int) map_.info.height * map_.info.resolution / 2;

    // only update if the robot has moved more than MIN_ROBOT_MOVEMENT_DISTANCE since last update (to improve
    // performance)
    if (distance(map_origin.point, map_.info.origin.position) > MIN_ROBOT_MOVEMENT_DISTANCE) {
        // transform map cells
        vector<int8_t> newdata(map_.data.size(), MAP_DEFAULT_VALUE);

        // get transformation from old map to new.
        int transform_x = (map_origin.point.x - map_.info.origin.position.x) / map_.info.resolution;
        int transform_y = (map_origin.point.y - map_.info.origin.position.y) / map_.info.resolution;

        for (size_t i = 0; i < map_.data.size(); ++i) {
            if (map_.data[i] != -1) {
                int x, y, new_x, new_y;
                x = i % map_.info.width;
                y = i / map_.info.width;
                new_x = x - transform_x;
                new_y = y - transform_y;

                // (int)-casts to supress comparison warning (no danger of overflow here)
                if (new_x >= 0 && new_x < (int)map_.info.width && new_y >= 0 && new_y < (int)map_.info.height) {
                    int offset = new_y * map_.info.width + new_x;
                    newdata[offset] = map_.data[i];
                }
            }
        }

        map_.info.origin.position = map_origin.point;
        map_.data = newdata;
    }
}

double TerrainClassifier::distance(geometry_msgs::Point a, geometry_msgs::Point b)
{
    return sqrt( pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2) );
}

void TerrainClassifier::saveScanCallback(const std_msgs::EmptyConstPtr &msg)
{
    save_next_scan_ = true;
}
