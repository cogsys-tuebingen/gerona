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
    float t1,t2,t3,r11,r12,r13,r21,r22,r23,r31,r32,r33;
    bool hasLaser2Base_;
    bool isStaticTransform_;

    Scan2CloudHelper():
        nodeP_("~")
    {

        // init parameter with a default value
        nodeP_.param<std::string>("baseFrame",baseFrame_,"base_link");
        nodeP_.param<bool>("isStaticTransform",isStaticTransform_,false);


        scan_sub_ = nodeG_.subscribe<sensor_msgs::LaserScan>(
                    "scan", 1, &Scan2CloudHelper::scanCallback, this);

        point_cloud_publisher_ = nodeG_.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1, false);

        t1 = 0;
        t2 = 0;
        t3 = 0;
        r11 = 1.0f;
        r12 = 0;
        r13 = 0;
        r21 = 0;
        r22 = 1.0f;
        r23 = 0;
        r31 = 0;
        r32 = 0;
        r33 = 1.0f;
        hasLaser2Base_ = false;

    }


    void SetupMatrices(tf::Transform &transform)
    {
        tf::Matrix3x3 rotMat(transform.getRotation());

        r11 = rotMat[0][0];
        r12 = rotMat[0][1];
        r13 = rotMat[0][2];
        r21 = rotMat[1][0];
        r22 = rotMat[1][1];
        r23 = rotMat[1][2];
        r31 = rotMat[2][0];
        r32 = rotMat[2][1];
        r33 = rotMat[2][2];

        tf::Vector3 nTrans = transform.getOrigin();

        t1 = nTrans.x();
        t2 = nTrans.y();
        t3 = nTrans.z();

        hasLaser2Base_ = true;


    }
    void CreateCloudXYZ(sensor_msgs::PointCloud2  &cloud, int size)
    {
        int pointSize = sizeof(float)*3;

        cloud.header.frame_id = baseFrame_;
        cloud.header.stamp = ros::Time::now();
        cloud.height = 1;
        cloud.width = size;
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        cloud.point_step = pointSize;
        cloud.row_step = cloud.point_step*cloud.width;

        sensor_msgs::PointField fieldx;
        fieldx.count = 1;
        fieldx.name =  'x';
        fieldx.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldx.offset = 0;
        sensor_msgs::PointField fieldy;
        fieldy.count = 1;
        fieldy.name =  'y';
        fieldy.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldy.offset = 4;
        sensor_msgs::PointField fieldz;
        fieldz.count = 1;
        fieldz.name =  'z';
        fieldz.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldz.offset = 8;

        cloud.fields.push_back(fieldx);
        cloud.fields.push_back(fieldy);
        cloud.fields.push_back(fieldz);

        cloud.data.resize(cloud.point_step*cloud.width);

    }

    void CreateCloudXYZI(sensor_msgs::PointCloud2  &cloud, int size)
    {
        int pointSize = sizeof(float)*4;

        cloud.header.frame_id = baseFrame_;
        cloud.header.stamp = ros::Time::now();
        cloud.height = 1;
        cloud.width = size;
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        cloud.point_step = pointSize;
        cloud.row_step = cloud.point_step*cloud.width;

        sensor_msgs::PointField fieldx;
        fieldx.count = 1;
        fieldx.name =  'x';
        fieldx.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldx.offset = 0;
        sensor_msgs::PointField fieldy;
        fieldy.count = 1;
        fieldy.name =  'y';
        fieldy.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldy.offset = 4;
        sensor_msgs::PointField fieldz;
        fieldz.count = 1;
        fieldz.name =  'z';
        fieldz.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldz.offset = 8;
        sensor_msgs::PointField fieldi;
        fieldi.count = 1;
        fieldi.name =  "intensity";
        fieldi.datatype =  sensor_msgs::PointField::FLOAT32;
        fieldi.offset = 12;

        cloud.fields.push_back(fieldx);
        cloud.fields.push_back(fieldy);
        cloud.fields.push_back(fieldz);
        cloud.fields.push_back(fieldi);

        cloud.data.resize(cloud.point_step*cloud.width);


    }


    void ProjectLaserSimple(sensor_msgs::PointCloud2 &cloud,const sensor_msgs::LaserScan& scan)
    {

        float *cdata = (float*)&cloud.data[0];

        float curAngle = scan.angle_min;

        float trange = 0;
        int curIdx = 0;
        float tx,ty,tz;
        float range_cutoff = scan.range_max * 0.99f;
        for (int tl = 0; tl < scan.ranges.size();tl++)
        {
            trange = scan.ranges[tl];

            if (trange > 0.0f && trange < range_cutoff && !std::isnan(trange))
            {
                tx = cos(curAngle)*trange;
                ty = sin(curAngle)*trange;
                tz = 0;

                cdata[curIdx] = tx*r11+ty*r12+tz*r13+t1;
                cdata[curIdx+1] = tx*r21+ty*r22+tz*r23+t2;
                cdata[curIdx+2] = tx*r31+ty*r32+tz*r33+t3;

            }

            curIdx += 3;
            curAngle += scan.angle_increment;
        }


    }


    void ProjectSimple(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        if (!hasLaser2Base_)
        {
            if (!tf_listener_.canTransform(baseFrame_, scan_in->header.frame_id,ros::Time(0)))
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "Scan2Cloud: cannot lookup transform frame " << scan_in->header.frame_id << " to " << baseFrame_);
                return;
            }
             tf::StampedTransform trans;
             tf_listener_.lookupTransform(baseFrame_, scan_in->header.frame_id, ros::Time(0), trans);
             SetupMatrices(trans);

        }
        sensor_msgs::PointCloud2 cloud;
        CreateCloudXYZ(cloud ,scan_in->ranges.size());
        ProjectLaserSimple(cloud, *scan_in);
        point_cloud_publisher_.publish(cloud);

    }



    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        if (isStaticTransform_)
        {
            ProjectSimple(scan_in);
            return;
        }
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
