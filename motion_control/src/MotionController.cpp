#include <geometry_msgs/Point32.h>
#include "MotionController.h"


void MotionController::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  laser_scan_=*scan;
}

bool MotionController::checkCollision( double course, double threshold, double width, double length )
{
  return laser_env_.CheckCollision(laser_scan_.ranges,laser_scan_.angle_min,laser_scan_.angle_max, course, width, length, threshold );
}
