#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include <Eigen/Core>
#include <sensor_msgs/LaserScan.h>
#include "Global.h"
#include "motion_control/MotionGoal.h"
#include "motion_control/MotionFeedback.h"
#include "motion_control/MotionResult.h"
#include <utils/LibRobot/LaserEnvironment.h>
#include <string>
#include <ros/ros.h>

using namespace Eigen;
using namespace motion_control;
class MotionControlNode;
class MotionController
{
public:
  virtual void start ()=0;
  virtual void stop ()=0;
  virtual int getType ()=0;

  /**
    @return state
    */
  virtual int execute (MotionFeedback& fb, MotionResult& result)=0;
  virtual void configure ()=0;
  virtual void setGoal (const motion_control::MotionGoal& goal)=0;
  virtual void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);

protected:
  bool checkCollision(double course,double threshold);
  sensor_msgs::LaserScan laser_scan_;
  LaserEnvironment laser_env_;
};


#endif // MOTIONCONTROLLER_H
