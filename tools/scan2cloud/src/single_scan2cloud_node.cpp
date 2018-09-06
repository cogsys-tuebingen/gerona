#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>



class Scan2CloudHelper
{
public:

    ros::NodeHandle nodeG_;
    ros::NodeHandle nodeP_;
    ros::Subscriber scan_sub_;
    tf::TransformListener tf_listener_;
    laser_geometry::LaserProjection projector_;
    ros::Publisher point_cloud_publisher_;
    std::string baseFrame_;


    Scan2CloudHelper():
        nodeP_("~")
    {

        // init parameter with a default value
        nodeP_.param<std::string>("baseFrame",baseFrame_,"base_link");


        scan_sub_ = nodeG_.subscribe<sensor_msgs::LaserScan>(
                    "scan", 1, &Scan2CloudHelper::scanCallback, this);

        point_cloud_publisher_ = nodeG_.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1, false);

    }


    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        ros::Duration wait_tf_timeout = ros::Duration(0.02);

        try{
            if(!tf_listener_.waitForTransform(
                        scan_in->header.frame_id,
                        baseFrame_,
                        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                        wait_tf_timeout)){
                ROS_ERROR_STREAM_THROTTLE(1.0, "cannot transform "  << ( "front")
                                          << " from frame " << scan_in->header.frame_id << " to " << baseFrame_);
                return;
            }
            // as (at least in the simulation) there are end-of-range scans slightly below the
            // range_max value, move the cutof to 99% of the range.
            double range_cutoff = scan_in->range_max * 0.99;

            sensor_msgs::PointCloud2 cloud_front_;

            projector_.transformLaserScanToPointCloud(baseFrame_, *scan_in, cloud_front_, tf_listener_, range_cutoff);

            cloud_front_.header.frame_id = baseFrame_;
            point_cloud_publisher_.publish(cloud_front_);
        } catch(...) {
            // do nothing
        }
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2cloud", ros::init_options::NoSigintHandler);

    Scan2CloudHelper scanHelper;

    ros::Rate r(60);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
