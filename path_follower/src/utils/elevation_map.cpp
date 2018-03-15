
/// HEADER
#include <path_follower/utils/elevation_map.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <tf/tf.h>



ElevationMap::ElevationMap()
    : elevationMap(nullptr)
{}

ElevationMap::ElevationMap(const EMap &c)
    : elevationMap(c)
{
}


bool ElevationMap::empty() const
{
    return elevationMap == nullptr;
}

void ElevationMap::clear()
{
    elevationMap = nullptr;
}

cv::Mat  ElevationMap::toCVMat() const
{

    return empty()?cv::Mat() : cv_bridge::toCvShare(elevationMap,"")->image;

}


ros::Time ElevationMap::getStamp() const
{    
    return elevationMap->header.stamp;
}

std::string ElevationMap::getFrameId() const
{
    return elevationMap->header.frame_id;
}
