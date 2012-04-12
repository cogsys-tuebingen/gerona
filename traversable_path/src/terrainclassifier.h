#ifndef DISPLAY_LASER_DATA_H
#define DISPLAY_LASER_DATA_H

#include <string>
#include <vector>
#include <list>
#include <boost/circular_buffer.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "laser_geometry/laser_geometry.h"
#include "dynamic_reconfigure/server.h"

#include "pointclassification.h"
#include "traversable_path/LaserScanClassification.h"
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
    typedef uint8_t classification;

    //! Default path/name of the range calibration file
    const static std::string DEFAULT_RANGE_CALIBRATION_FILE;

    //! ROS node handle.
    ros::NodeHandle node_handle_;

    /**
     * @brief Publishes normalized laser data (using intensity as traverability-indicator)
     * @todo This is only for debugging and should be removed in later versions.
     */
    ros::Publisher publish_normalized_;

    //! Publishes the point classification for the laser scan data
    ros::Publisher publish_path_points_;
    //! Subscribes for laser scans.
    ros::Subscriber subscribe_laser_scan_;
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
    float avg(boost::circular_buffer<float> &xs);

    //! Classifies the points of the given scan.
    std::vector<bool> detectObstacles(sensor_msgs::LaserScan data, std::vector<float> &out);

    // ! removes single peaks caused only by intensity (which are in most cases no untraversable areas).
    //void removeSingleIntensityPeaks(std::vector<PointClassification> &segments);

    //! Callback for dynamic reconfigure.
    void dynamicReconfigureCallback(Config &config, uint32_t level);
};

const std::string TerrainClassifier::DEFAULT_RANGE_CALIBRATION_FILE = ros::package::getPath(ROS_PACKAGE_NAME)
                                                             + std::string("/rangecalibration.yaml");

#endif // DISPLAY_LASER_DATA_H
