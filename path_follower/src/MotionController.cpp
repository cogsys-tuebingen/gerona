#include <geometry_msgs/Point32.h>
#include "MotionController.h"
#include <utils_general/MathHelper.h>
#include <opencv2/opencv.hpp>

#define DEBUG 0

MotionController::MotionController()
    : use_obstacle_map_(false),
      sonar_collision_( false ),
      sonar_stamp_( ros::Time::now())
{

}

void MotionController::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    laser_scan_=*scan;
}

void MotionController::obstacleMapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    obstacle_map_=*map;
    obstacle_detector_.gridMapCallback(map);
    vfh_.setMap(*map);
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

bool MotionController::checkCollision(double course_angle, double box_length, double box_width, double curve_enlarge_factor)
{
    //    if ((ros::Time::now() - sonar_stamp_).toSec() > 2 )
    //        sonar_collision_ = false;

    // Backwards?
    /*if ( fabs( MathHelper::AngleDelta( course, 0  )) > 0.5*M_PI ) {
        return sonar_collision_;
    }*/

    if(use_obstacle_map_) {
        return obstacle_detector_.isObstacleAhead(box_width, box_length, course_angle, curve_enlarge_factor);

        /*
        if(obstacle_map_.data.empty()) {
            ROS_WARN("no obstacle map received!!!!");
            return true;
        }

        bool collision = false;

        double res = obstacle_map_.info.resolution;

        // TODO: replace with something better :)
        float beta = course_angle;
        float w = box_width / res;
        float length = curve_enlarge_factor / res;
        float thresh = box_length / res;
        // corner points of the parallelogram
        float ax,ay,bx,by,cx,cy;

        float sbeta=std::sin(beta);
        float cbeta=std::cos(beta);
        ay=w/2.0f;
        ax=0.0f;
        by=-w/2.0f;
        bx=0.0f;
        if (beta > 0) {
            ay += length*sbeta;
        } else if (beta < 0) {
            by += length*sbeta;
        }
        cy=ay+thresh*sbeta;
        cx=ax+thresh*cbeta;

        cv::Point2f p1, p2, p3, p4;
        p1.y = ay;  p1.x = ax;
        p2.y = by;  p2.x = bx;
        p3.y = cy;  p3.x = cx;
        p4.y = p3.y + p2.y - p1.y;
        p4.x = p3.x + p2.x - p1.x;

        cv::Mat map_cv(obstacle_map_.info.height, obstacle_map_.info.width, CV_8SC1, obstacle_map_.data.data());
        cv::Mat mask(obstacle_map_.info.height, obstacle_map_.info.width, CV_8UC1, cv::Scalar::all(0));
#if DEBUG
        cv::Mat debug;
        map_cv.copyTo(debug);
#endif

        cv::Point2f o(-obstacle_map_.info.origin.position.x / res,
                    -obstacle_map_.info.origin.position.y / res);


        std::vector<std::vector<cv::Point> > contours;
        contours.push_back(std::vector<cv::Point>());
        contours[0].push_back(o+p1);
        contours[0].push_back(o+p2);
        contours[0].push_back(o+p4);
        contours[0].push_back(o+p3);
        cv::drawContours(mask, contours, 0, cv::Scalar(255), CV_FILLED);


        double offset = 5.0;

        cv::Point2f tl, br;
        tl.x = std::min(p1.x, std::min(p2.x, std::min(p3.x, p4.x))) - offset;
        tl.y = std::min(p1.y, std::min(p2.y, std::min(p3.y, p4.y))) - offset;
        br.x = std::max(p1.x, std::max(p2.x, std::max(p3.x, p4.x))) + offset;
        br.y = std::max(p1.y, std::max(p2.y, std::max(p3.y, p4.y))) + offset;

        cv::Rect rect;
        rect.x = o.x + tl.x;
        rect.y = o.y + tl.y;
        rect.width = br.x - tl.x;
        rect.height = br.y - tl.y;
//        cv::rectangle(mask, rect, cv::Scalar(127), 2);

        for(int dy = 0; dy < rect.height; ++dy) {
            for(int dx = 0; dx < rect.width; ++dx) {
                int x = rect.x + dx;
                int y = rect.y + dy;
                bool check = mask.at<uchar>(y,x) == 255;
                if(check) {
                    const char& val = map_cv.at<char>(y,x);
                    if(val > 70 && val <= 100) {
                        collision = true;
#if DEBUG
                        debug.at<char>(y,x) = 127;
#endif
                    }
#if DEBUG
                    else {
                        debug.at<char>(y,x) = -127;
                    }
#endif
                }
            }
        }
#if DEBUG
        cv::flip(debug, debug, 0);
        cv::imshow("obstacles", debug);
        cv::waitKey(30);
#endif
        return collision;
        */
    } else {
        return laser_env_.CheckCollision(laser_scan_.ranges,laser_scan_.angle_min,laser_scan_.angle_max, course_angle, box_width,
                                         curve_enlarge_factor, box_length);
    }
}

bool MotionController::simpleCheckCollision(float box_width, float box_length)
{
    for (size_t i=0; i < laser_scan_.ranges.size(); ++i) {
        // project point to carthesian coordinates
        float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
        float px = laser_scan_.ranges[i] * cos(angle);
        float py = laser_scan_.ranges[i] * sin(angle);


        /* Point p is inside the rectangle, if
         *    p.x in [-width/2, +width/2]
         * and
         *    p.y in [0, length]
         */

        if ( py >= -box_width/2 &&
             py <=  box_width/2 &&
             px >= 0 &&
             px <= box_length )
        {
            return true;
        }
    }

    return false;
}
