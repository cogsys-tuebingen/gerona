#include <geometry_msgs/Point32.h>
#include "MotionController.h"
#include <utils_general/MathHelper.h>
#include <opencv2/opencv.hpp>

#define DEBUG 0

MotionController::MotionController()
    : sonar_collision_( false ),
      sonar_stamp_( ros::Time::now())
{
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
