#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include "point_types.h"
#include "pointclassification.h"
#include "mapprocessor.h"
#include "traversable_path/classify_terrainConfig.h"

/**
 * @brief Subscribes for the laser scans and classifies them.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class TerrainClassifier
{
public:
    TerrainClassifier();
    ~TerrainClassifier();

private:
    typedef traversable_path::classify_terrainConfig Config;
    typedef pcl::PointCloud<PointXYZRGBT> PointCloudXYZRGBT;

    //! Default path/name of the range calibration file
    const static std::string DEFAULT_RANGE_CALIBRATION_FILE;

    //! ROS node handle.
    ros::NodeHandle node_handle_;

    /**
     * @brief Publishes normalized laser data (using intensity as traverability-indicator)
     * @todo This is only for debugging and should be removed in later versions.
     */
    ros::Publisher publish_normalized_;
    ros::Publisher publish_normalized_diff;

    //! Publisher for the classification point cloud
    ros::Publisher publish_classification_cloud_[4];
    //! Subscribes for the tilted laser scans.
    message_filters::Subscriber<sensor_msgs::LaserScan> subscriber_laser_scan_[4];
    tf::MessageFilter<sensor_msgs::LaserScan> *message_filter_tilted_scan_[4];
    //! Subscriber for the horizontal front scanner
    message_filters::Subscriber<sensor_msgs::LaserScan> subscriber_front_scan_;
    tf::MessageFilter<sensor_msgs::LaserScan> *message_filter_front_scan_;
    //! Listener for tf data.
    tf::TransformListener tf_listener_;
    //! Registers calibration service.
    ros::ServiceServer calibration_service_;
    //! projects laser data to carthesian frame.
    laser_geometry::LaserProjection laser_projector_;
    //! dynamic reconfigure server.
    dynamic_reconfigure::Server<Config> reconfig_server_;

    //! Name of the range calibration file
    std::string range_calibration_file_;
    //! True if already calibrated, false if not.
    bool is_calibrated_;
    //! Range data of a (preferably) perfekt plane, to calibrate the laser data.
    std::vector<std::vector<float> > plane_ranges_;
    //! Buffer of the last few scans.
    boost::circular_buffer< std::vector<PointClassification> > scan_buffer_;

    //! dynamic reconfigure values.
    Config config_;

    static const int8_t MAP_DEFAULT_VALUE = 50;
    nav_msgs::OccupancyGrid map_;
    ros::Publisher publish_map_;
    MapProcessor map_processor_;
    ros::Timer map_publish_timer_;

    bool save_next_scan_;
    ros::Subscriber subscribe_save_scan_;
    void saveScanCallback(const std_msgs::EmptyConstPtr &msg);

    /**
     * @brief Classifies the laser scan points.
     *
     * This is the callback function for the laser scans. It classifies the points, transforms them to the carthesian
     * frame of the laser and publishes the result as ScanClassification message.
     */
    void classifyLaserScan(const sensor_msgs::LaserScanConstPtr &msg, uint layer);

    //! Callback for the horizontal front scanner (for obstacle avoidance).
    void frontScanCallback(const sensor_msgs::LaserScanConstPtr &msg);

    //! Calibrates the laser, assuming the current scan shows a flat plane without obstacles.
    bool calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /**
     * @brief Smoothes the curve describted by data.
     */
    static std::vector<float> smooth(std::vector<float> data, const unsigned int num_values);

    /**
     * @brief Calculates average of the elements of a float list.
     * @param A list of float-values.
     * @return Average of the list-values.
     */
    static float avg(const boost::circular_buffer<float> &xs);

    //! Classifies the points of the given scan.
    std::vector<PointClassification> detectObstacles(const sensor_msgs::LaserScan &data, uint layer, std::vector<float> &out);

    //! Calculate for each element in data the variance of the window around the element.
    static std::vector<float> calcVariances(const std::vector<float> &data, unsigned int window_size);

    static float variance(const boost::circular_buffer<float> &window);

    /**
     * @brief Check neighbourhood of each point for some features.
     *
     * @param Classification of the points. This method will set flags to some points, depending on their neighbours.
     */
    void checkPointNeighbourhood(std::vector<PointClassification> *scan_classification) const;

    /**
     * @brief Does some feature checks that requires a point cloud.
     *
     * This method calls all feature checks that require the scan data as a point cloud (e.g. width of the path).
     * Please note: A requirement is, that the points are already classified! However the classification of some points
     * may be changed by calling this method (e.g. to narrow traversable segments will be made untraversable).
     *
     * @param cloud Point cloud of the laser scan. The points already have to be classified. Classification of the
     *              points may be changed by this method.
     */
    void classifyPointCloud(PointCloudXYZRGBT *cloud) const;

    /**
     * @brief Checks a single traversable segment some features.
     *
     * The given segment is checked for the feaures width and slope. If the segment is too narrow or too steep, it is
     * marked as untraversable.
     * @param begin Start of the traversable segment
     * @param end   End of the traversable segment
     */
    void checkTraversableSegment(PointCloudXYZRGBT::iterator begin,
                                 PointCloudXYZRGBT::iterator end) const;

    //! Callback for dynamic reconfigure.
    void dynamicReconfigureCallback(Config &config, uint32_t level);

    void laserScanToCloud(const sensor_msgs::LaserScanConstPtr &scan, const std::vector<PointClassification> &traversable,
                          PointCloudXYZRGBT *cloud);

    bool scanToFile(std::string filename, const sensor_msgs::LaserScan &scan);



    //! Update the map with a cloud of classified points.
    void updateMap(PointCloudXYZRGBT cloud);

    //! Publish a filtered version of the map.
    void publishMap(const ros::TimerEvent&);

    /**
     * @brief Move the map if necessary.
     *
     * Move the map to make the robot be in the center. Each cell is translated, losing information about cells that
     * are translated to a point outside the map.
     * The moving is only done, if the robot has moved at least a minium distance from the center, to reduce the
     * computational cost.
     */
    void moveMap();

    //! Get distance between a and b.
    static double distance(geometry_msgs::Point a, geometry_msgs::Point b);
};

#endif // DISPLAY_LASER_DATA_H
