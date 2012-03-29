/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com

   @author Karsten Bohlmann
   @date   1/15/2012
   @file   FixedDriver.h

*/

#ifndef PATTERNDRIVER_H
#define PATTERNDRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <utils/LibRobot/LaserEnvironment.h>
#include "StatsEstimator.h"

#include "Stopwatch.h"
#include "MotionController.h"

#include <list>
class MotionControlNode;
class PatternDriver : public MotionController
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PatternDriver(ros::Publisher& cmd_pub,MotionControlNode *node);
  virtual void start ();
  virtual void stop ();
  virtual int getType () {
    return motion_control::MotionGoal::MOTION_FIXED_PARAMS;
  }
  virtual int execute (MotionFeedback& fb, MotionResult& result);
  virtual void configure ();
  virtual void setGoal (const motion_control::MotionGoal& goal);

protected:
  bool update( Vector3d& slam_pose );
  void nextWaypoint( Vector3d& slam_pose );
  void generateWaypoints( double delta_f );
  void generateLine( const Vector3d &line_start,
                     const double length,
                     const double wp_dist,
                     Vector3d &line_end );
  void generateCurve( const Vector3d &curve_start,
                      const double radius,
                      const double wp_dist,
                      const bool positive,
                      Vector3d &curve_end );

private:
  MotionControlNode *node_;
  ros::Publisher& cmd_pub_;
  double cmd_v_;
  double cmd_front_rad_, cmd_rear_rad_;

  double default_v_;
  int state_;
  Stopwatch move_timer_;
  Vector3d start_pose_, last_pose_;
  double beta_;
  double driven_dist_;
  double dist_measure_threshold_;

  std::list<Vector3d, Eigen::aligned_allocator<Vector3d> > waypoints_;

};
#endif
