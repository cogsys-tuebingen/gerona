#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


class ScanConverter {
     public:
        ScanConverter();
        void scanCallback_front(const sensor_msgs::LaserScan::ConstPtr& scan_in);
        void scanCallback_back(const sensor_msgs::LaserScan::ConstPtr& scan_in);
        void spin();
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_back_;
        ros::Subscriber scan_sub_front_;
        sensor_msgs::PointCloud2 cloud_front_;
        sensor_msgs::PointCloud2 cloud_back_;
        sensor_msgs::PointCloud2 cloud_total_;

        bool cbCloud_;

         // ros params
        std::string baseFrame_;
        std::string scanTopic_front_;
        std::string scanTopic_back_;
        std::string cloudTopic_;

        double cloudFilterMean_;
        double cloudFilterStdD_;


        void mergeSensorMsgsPointCloud2();
        void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOut);
        void updateParameter();

};
