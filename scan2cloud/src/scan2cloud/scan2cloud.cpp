#include "scan2cloud.h"

ScanConverter::ScanConverter():node_("~"){
    // init parameter with a default value
    node_.param<std::string>("baseFrame",baseFrame_,"/base_link");

    node_.param<double>("cloudFilterMean",cloudFilterMean_,50.0);
    node_.param<double>("cloudFilterStdD",cloudFilterStdD_,1.0);


    scan_sub_front_ = node_.subscribe<sensor_msgs::LaserScan>(
                "/scan/front/filtered", 1, boost::bind(&ScanConverter::scanCallback, this, _1, false));
    scan_sub_back_  = node_.subscribe<sensor_msgs::LaserScan>(
                "/scan/back/filtered", 1, boost::bind(&ScanConverter::scanCallback, this ,_1, true));
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud/total", 1, false);
}

void ScanConverter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in, bool is_back)
{
    ros::Duration wait_tf_timeout = is_back ? ros::Duration(0.0) : ros::Duration(0.1);

    try{
        if(!tfListener_.waitForTransform(
                    scan_in->header.frame_id,
                    "/base_link",
                    scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                    wait_tf_timeout)){
            ROS_INFO("ignore scan");
            return;
        }
        // as (at least in the simulation) there are end-of-range scans slightly below the
        // range_max value, move the cutof to 99% of the range.
        double range_cutoff = scan_in->range_max * 0.99;
        if (is_back) {
            projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud_back_, tfListener_, range_cutoff);
            cbScanback_ = true;
        } else {
            projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud_front_, tfListener_, range_cutoff);
            cbScanfront_ = true;
        }
    } catch(...) {
        // do nothing
    }
}

void ScanConverter::spin()
{

    cbScanfront_ = false;
    cbScanback_ = false;
    ros::Rate loopRate(50);
    while(ros::ok()){
        //ros::spinOnce();
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));

        if(cbScanfront_ || cbScanback_){
            this->mergeSensorMsgsPointCloud2();
            point_cloud_publisher_.publish(cloud_total_);
        }

        //this->updateParameter();

        loopRate.sleep();
    }
}

void ScanConverter::mergeSensorMsgsPointCloud2()
{
    pcl::PointCloud<pcl::PointXYZ> combined_pcl;
    pcl::PointCloud<pcl::PointXYZ> output_pcl_front;
    pcl::PointCloud<pcl::PointXYZ> output_pcl_back;

    if(cbScanfront_){
        pcl::fromROSMsg(cloud_front_,output_pcl_front);
        combined_pcl = output_pcl_front;
    }
    if(cbScanback_){
        pcl::fromROSMsg(cloud_back_,output_pcl_back);
        if(cbScanfront_){
            combined_pcl += output_pcl_back;
        }else{
            combined_pcl = output_pcl_back;
        }
    }

    if(cbScanfront_ || cbScanback_){
        pcl::toROSMsg(combined_pcl, cloud_total_);
        cloud_total_.header.frame_id = baseFrame_;
    }

}

void ScanConverter::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOut){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloudIn);
    sor.setMeanK (cloudFilterMean_);
    sor.setStddevMulThresh (cloudFilterStdD_);
    sor.filter (*cloud_filtered);

    cloudOut = cloud_filtered;

    cloudOut = cloudIn;


}
