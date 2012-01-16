/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/15/2012
   @file   FixedDriver.h

*/ 

#ifndef FIXEDDRIVER_H
#define FIXEDDRIVER_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <utils/LibRobot/LaserEnvironment.h>
#include "StatsEstimator.h"

#include "Stopwatch.h"
#include "MotionController.h"
class FixedDriver : public MotionController
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FixedDriver(ros::Publisher& cmd_pub,ros::NodeHandle& node);
  virtual void start ();
  virtual void stop ();
  virtual int getType () {
    return motion_control::MotionGoal::MOTION_FIXED_PARAMS;
  }
  virtual int execute (MotionFeedback& fb, MotionResult& result);
  virtual void configure (ros::NodeHandle &node);
  virtual void setGoal (const motion_control::MotionGoal& goal);

private:

  ros::Publisher&     cmd_pub_;
  double cmd_v_;
  double cmd_front_rad_,cmd_rear_rad_;

  double default_v_;
  int state_;
  Stopwatch move_timer_;
  Vector3d start_pose_;
  double beta_;

};

#endif // FIXEDDRIVER_H
