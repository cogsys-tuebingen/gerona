#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H


#include <sensor_msgs/LaserScan.h>
#include <utils_general/Global.h>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <path_msgs/FollowPathAction.h>
#include <nav_msgs/OccupancyGrid.h>

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
  virtual void sonarCallback(const sensor_msgs::PointCloudConstPtr& data);
  void setFilteredSpeed( const float speed ) {
      filtered_speed_ = speed;
  }



protected:
  float filtered_speed_;

  bool sonar_collision_;
  ros::Time sonar_stamp_;

  float getFilteredSpeed() const {
      return filtered_speed_;
  }

};


#endif // MOTIONCONTROLLER_H
