#include <geometry_msgs/Point32.h>
#include "MotionController.h"

MotionController::MotionController()
    : backwards_collision_( false ),
      sonar_stamp_( ros::Time::now())
{

}

void MotionController::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    laser_scan_=*scan;
}

void MotionController::sonarCallback(const sensor_msgs::PointCloudConstPtr &data)
{
    backwards_collision_ = false;
    sonar_stamp_ = ros::Time::now();

    size_t size = data->points.size();
    for ( size_t i = 0; i < size; ++i ) {
        if ( data->points[i].x < 0
             && sqrt( pow( data->points[i].x, 2 ) + pow( data->points[i].y, 2 )) < 0.1 ) {
            backwards_collision_ = true;
            return;
        }
    }
}

bool MotionController::checkCollision( double course, double threshold, double width, double length )
{
    if ((ros::Time::now() - sonar_stamp_).toSec() > 2 )
        backwards_collision_ = false;

    // Backwards?
    if ( course > 0.5*M_PI || course < -0.5*M_PI ) {
        return backwards_collision_;
    }

    return laser_env_.CheckCollision(laser_scan_.ranges,laser_scan_.angle_min,laser_scan_.angle_max, course, width, length, threshold );
}
