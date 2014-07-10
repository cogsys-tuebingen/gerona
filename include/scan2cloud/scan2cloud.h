#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class ScanConverter {
     public:
        ScanConverter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};
