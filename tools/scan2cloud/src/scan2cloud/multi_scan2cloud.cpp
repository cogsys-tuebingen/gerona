

#include "../../include/scan2cloud/multi_scan2cloud.h"



void ToPoints(const sensor_msgs::LaserScan &scan, const std::vector<bool> &scan_mask, std::vector<tf::Point> &points)
{
    double curAngle = scan.angle_min;
    points.clear();
    if (points.capacity() < scan.ranges.size()) points.reserve(scan.ranges.size());

    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        double rd = (double)scan.ranges[i];
        tf::Point p(cos(curAngle)*rd,sin(curAngle)*rd,0);
        if (scan_mask[i] && !std::isnan(rd)) points.push_back(p);
        curAngle += scan.angle_increment;

    }

}

void ToPoints(const sensor_msgs::LaserScan &scan, std::vector<tf::Point> &points)
{
    double curAngle = scan.angle_min;
    points.clear();
    if (points.capacity() < scan.ranges.size()) points.reserve(scan.ranges.size());

    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        double rd = (double)scan.ranges[i];
        tf::Point p(cos(curAngle)*rd,sin(curAngle)*rd,0);
        if (!std::isnan(rd)) points.push_back(p);
        curAngle += scan.angle_increment;

    }

}


inline float tukey(const float &x, const float &k)
{
    if (std::abs(x) > k) return 0.0f;
    const float xok = x/k;
    const float d = (1.0f-xok*xok);
    return d*d;
}

inline float normSqr(const tf::Point &v) {return(v.dot(v));}
inline float norm(const tf::Point &v) {return sqrt(normSqr(v));}


float GetWeight(const std::vector<tf::Point> &points, const float &k, const int &windowSize,const int &idx)
{
    int start = idx-windowSize;
    int end = idx+windowSize+1;

    start = std::max(0,start);
    end = std::min((int)points.size()-1,end);

    const tf::Point &pc = points[idx];

    float sumW = 0;
    for (int i = start; i < end; ++i)
    {
        sumW += tukey(norm(points[i]-pc),k);
    }

    return sumW;
}


void NoiseFilterDist(const std::vector<tf::Point> &in_points, const float &threshold, const float &k, const int &windowSize, std::vector<tf::Point> &out_points)
{
    out_points.clear();
    if (out_points.capacity() < in_points.size()) out_points.reserve(in_points.size());
    for (int i = 0; i < in_points.size(); ++i)
    {
        const float w = GetWeight(in_points,norm(in_points[i])*k,windowSize,i);
        if (w > threshold) out_points.push_back(in_points[i]);
    }

}
void NoiseFilter(const std::vector<tf::Point> &in_points, const float &threshold, const float &k, const int &windowSize, std::vector<tf::Point> &out_points)
{
    out_points.clear();
    if (out_points.capacity() < in_points.size()) out_points.reserve(in_points.size());
    for (int i = 0; i < in_points.size(); ++i)
    {
        float w = GetWeight(in_points,k,windowSize,i);
        if (w > threshold) out_points.push_back(in_points[i]);
    }

}


ScanProcessor::ScanProcessor()
{
    tukey_k_ = 0.08f;
    use_dist_ = true;
    threshold_w_ = 1.5f;
    windowSize_ = 15;
}

void ScanProcessor::ProcessScan(const sensor_msgs::LaserScan &scan, const std::vector<bool> scanMask, std::vector<tf::Point> &out_points)
{
    ToPoints(scan,scanMask,points1_);
    if (use_dist_) NoiseFilterDist(points1_,threshold_w_,tukey_k_,windowSize_,points2_);
    else NoiseFilter(points1_,threshold_w_,tukey_k_,windowSize_,points2_);
    TransformCloud(points2_,scan.header.frame_id,scan.header.stamp,out_points);
    //CreateCloud(points1_,cloud);


}


void CreateEmptyCloud(int size, const std::string frame_id, const ros::Time stamp, sensor_msgs::PointCloud2 &cloud)
{
    cloud = sensor_msgs::PointCloud2();
    int pointSize = sizeof(float)*3;


    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
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



void ScanProcessor::CreateCloud(const std::vector<tf::Point> &p1, const std::vector<tf::Point> &p2, const std::string frame_id, const ros::Time stamp, sensor_msgs::PointCloud2 &cloud)
{
    CreateEmptyCloud((int)(p1.size()+p2.size()),frame_id,stamp,cloud);


    float *dstPtr = (float*) &cloud.data.front();

    for(int t = 0; t < p1.size();++t) {
        const tf::Point &point = p1[t];
        *dstPtr = point.x();
        ++dstPtr;
        *dstPtr = point.y();
        ++dstPtr;
        *dstPtr = point.z();
        ++dstPtr;
    }
    for(int t = 0; t < p2.size();++t) {
        const tf::Point &point = p2[t];
        *dstPtr = point.x();
        ++dstPtr;
        *dstPtr = point.y();
        ++dstPtr;
        *dstPtr = point.z();
        ++dstPtr;
    }
}

void ScanProcessor::CreateCloud(const std::vector<tf::Point> &obstacle_points, const std::string frame_id, const ros::Time stamp, sensor_msgs::PointCloud2 &cloud)
{

    CreateEmptyCloud(obstacle_points.size(),frame_id,stamp,cloud);


    float *dstPtr = (float*) &cloud.data.front();

    for(int t = 0; t < obstacle_points.size();++t) {
        const tf::Point &point = obstacle_points[t];
        *dstPtr = point.x();
        ++dstPtr;
        *dstPtr = point.y();
        ++dstPtr;
        *dstPtr = point.z();
        ++dstPtr;
    }

    //if (obstacle_points.size() > 0)memcpy((void*)&cloud.data.front(),&obstacle_points[0],cloud.point_step*cloud.width);


}

void ScanProcessor::TransformCloud(const tf::Transform& transform, const std::vector<tf::Point> &in, std::vector<tf::Point> &out)
{


    for(int t = 0; t < in.size();++t)
    {
        out.push_back( transform * in[t]);

    }

}

inline void AddPrefixSlash(const std::string &f, std::string &out)
{
    if (f.length() == 0)
    {
        out = "";
        return;
    }
    if (f[0] != '/')
    {
        out = "/"+f;
        return;
    }
    out = f;
}

inline bool FramesEqual(const std::string &f1, const std::string &f2)
{
    std::string p1;
    AddPrefixSlash(f1,p1);
    std::string p2;
    AddPrefixSlash(f2,p2);
    return p1.compare(p2) == 0;

}

void ScanProcessor::TransformCloud(const std::vector<tf::Point> &in, const std::string frame_id, const ros::Time stamp, std::vector<tf::Point> & out)
{
    out.clear();
    if (out.capacity() < in.size()) out.reserve(in.size());


    if(FramesEqual(frame_id,fixed_frame_)) {
        for(int t = 0; t < in.size();++t)
        {
            out.push_back(in[t]);

        }
    }
    else
    {

        if (!always_use_latest_transform_obstacles_)
        {

            try{

                tfListener_.waitForTransform(fixed_frame_, frame_id, stamp,ros::Duration(tf_timeout_));
                tf::StampedTransform transform;
                tfListener_.lookupTransform(fixed_frame_, frame_id, stamp,transform);
                TransformCloud(transform,in, out);

            } catch (tf::TransformException &ex) {
                ROS_WARN_STREAM("scan2cloud: TransformCloud Stamp: Could NOT transform " << frame_id << " to: "  << fixed_frame_ << " MSG: " << ex.what());
            }
            return;
        }
        else
        {
            try{

                tf::StampedTransform transform;
                tfListener_.lookupTransform(fixed_frame_, frame_id, ros::Time(0),transform);
                TransformCloud(transform,in, out);
            } catch (tf::TransformException &ex) {
                ROS_WARN_STREAM("scan2cloud: TransformCloud Stamp: Could NOT transform " << frame_id << " to: "  << fixed_frame_ << " MSG: " << ex.what());
                return;
            }

        }

        //ROS_ERROR("velocity_TT: Obstacle Cloud needs to be in fixed frame: %s is: %s",cloud.header.frame_id,opt_.fixed_frame());


    }





}


