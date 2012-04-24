#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include <string>
#include <vector>
#include <list>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include "point_types.h"
#include "pointclassification.h"
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

private:
    typedef traversable_path::classify_terrainConfig Config;

    //! Default path/name of the range calibration file
    const static std::string DEFAULT_RANGE_CALIBRATION_FILE;

    //! ROS node handle.
    ros::NodeHandle node_handle_;

    /**
     * @brief Publishes normalized laser data (using intensity as traverability-indicator)
     * @todo This is only for debugging and should be removed in later versions.
     */
    ros::Publisher publish_normalized_;

    //! Publisher for the classification point cloud
    ros::Publisher publish_classification_cloud_;
    //! Subscribes for laser scans.
    ros::Subscriber subscribe_laser_scan_;
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
    std::vector<float> plane_ranges_;
    //! Buffer of the last few scans.
    boost::circular_buffer< std::vector<PointClassification> > scan_buffer_;

    //! dynamic reconfigure values.
    Config config_;

    nav_msgs::OccupancyGrid map_;
    ros::Publisher publish_map_;
    void updateMap(pcl::PointCloud<PointXYZRGBT> cloud);
    void moveMap();
    double distance(geometry_msgs::Point a, geometry_msgs::Point b);

    /**
     * @brief Classifies the laser scan points.
     *
     * This is the callback function for the laser scans. It classifies the points, transforms them to the carthesian
     * frame of the laser and publishes the result as ScanClassification message.
     */
    void classifyLaserScan(const sensor_msgs::LaserScanPtr &msg);

    //! Calibrates the laser, assuming the current scan shows a flat plane without obstacles.
    bool calibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /**
     * @brief Smoothes the curve describted by data.
     */
    std::vector<float> smooth(std::vector<float> data, const unsigned int num_values);

    /**
     * @brief Calculates average of the elements of a float list.
     * @param A list of float-values.
     * @return Average of the list-values.
     */
    float avg(const boost::circular_buffer<float> &xs);

    //! Classifies the points of the given scan.
    std::vector<PointClassification> detectObstacles(const sensor_msgs::LaserScan &data, std::vector<float> &out);

    /**
     * @brief Check neighbourhood of each point for some features.
     *
     * @param Classification of the points. This method will set flags to some points, depending on their neighbours.
     */
    void checkPointNeighbourhood(std::vector<PointClassification> *scan_classification);

    /**
     * @brief Drop traversable segments, that are too narrow for the robot.
     *
     * @param points The classification and position of the points. The points of too narrow paths will be marked as
     *               untraversable within points.
     */
    void dropNarrowPaths(pcl::PointCloud<PointXYZRGBT> *cloud);

    //! Callback for dynamic reconfigure.
    void dynamicReconfigureCallback(Config &config, uint32_t level);
};

const std::string TerrainClassifier::DEFAULT_RANGE_CALIBRATION_FILE = ros::package::getPath(ROS_PACKAGE_NAME)
                                                             + std::string("/rangecalibration.yaml");

#endif // DISPLAY_LASER_DATA_H
