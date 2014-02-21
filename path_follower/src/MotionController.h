#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H


#include <sensor_msgs/LaserScan.h>
#include <utils_general/Global.h>
#include <utils_robot/LaserEnvironment.h>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <path_msgs/FollowPathAction.h>

class PathFollower;

class MotionController
{
public:
  MotionController();

  virtual void start ()=0;
  virtual void stop ()=0;
  virtual int getType ()=0;

  /**
    @return state
    */
  virtual int execute (path_msgs::FollowPathFeedback& fb, path_msgs::FollowPathResult& result)=0;
  virtual void configure ()=0;
  virtual void setGoal (const path_msgs::FollowPathGoal& goal)=0;
  virtual void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
  virtual void sonarCallback(const sensor_msgs::PointCloudConstPtr& data);
  void setFilteredSpeed( const float speed ) {
      filtered_speed_ = speed;
  }
  bool checkCollision(double course, double threshold, double width = 0.3);

  /**
   * @brief Check if there is an obstacle within a rectangular box in front of the robot.
   *
   * The box is placed in front of the laser and is defined by its width and length as displayed in the "figure" below:
   *
   *          +------------------+
   *  ##   ## |                  | |
   *  ####### |                  | width
   *  ####### |                  | |
   *  ##   ## |                  |
   *          +------------------+
   *   ^robot     <- length ->
   *
   *
   *
   * @param box_width Width of the box, which is checked for obstacles.
   * @param box_length Length of the box, which is checked for obstacles.
   * @return true, if there is an obstacle in the box.
   */
  bool simpleCheckCollision(float box_width, float box_length);

protected:
  float getFilteredSpeed() const {
      return filtered_speed_;
  }

  sensor_msgs::LaserScan laser_scan_;
  LaserEnvironment laser_env_;
  float filtered_speed_;
  bool sonar_collision_;
  ros::Time sonar_stamp_;
};


#endif // MOTIONCONTROLLER_H
