#include <geometry_msgs/Point32.h>
#include "MotionController.h"
#include <utils_general/MathHelper.h>

MotionController::MotionController()
    : sonar_collision_( false ),
      sonar_stamp_( ros::Time::now())
{

}

void MotionController::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    laser_scan_=*scan;
}

void MotionController::sonarCallback(const sensor_msgs::PointCloudConstPtr &data)
{
    sonar_collision_ = false;
    sonar_stamp_ = ros::Time::now();

    size_t size = data->points.size();
    for ( size_t i = 2; i < size; ++i ) { // Skip forward sensors
        if ( data->points[i].x > 0.02 && data->points[i].x < 0.1 ) {
            sonar_collision_ = true;
            return;
        }
    }
}

bool MotionController::checkCollision( double course, double threshold, double width, double length )
{
    if ((ros::Time::now() - sonar_stamp_).toSec() > 2 )
        sonar_collision_ = false;

    // Backwards?
    /*if ( fabs( MathHelper::AngleDelta( course, 0  )) > 0.5*M_PI ) {
        return sonar_collision_;
    }*/

    return laser_env_.CheckCollision(laser_scan_.ranges,laser_scan_.angle_min,laser_scan_.angle_max, course, width, length, threshold );
}
