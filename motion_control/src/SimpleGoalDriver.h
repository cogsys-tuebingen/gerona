#ifndef SIMPLEGOALDRIVER_H
#define SIMPLEGOALDRIVER_H
#include <ros/ros.h>
#include "LaserEnvironment.h"
#include "StatsEstimator.h"
#include "EncoderEstimator.h"
#include "Stopwatch.h"
#include "MotionController.h"
#include "DualPidCtrl.h"

class SimpleGoalDriver : public MotionController
{
public:
    SimpleGoalDriver(ros::Publisher& cmd_pub);
    virtual void start ();
    virtual void stop ();
    virtual int getType () {
      return motion_control::MotionGoal::MOTION_TO_GOAL;
    }
    virtual int execute (MotionFeedback& fb, MotionResult& result);
    virtual void configure (ros::NodeHandle &node);
    virtual void setGoal (const motion_control::MotionGoal& goal);

private:
    void publish ();
    double cmd_v_;
    double cmd_front_rad_,cmd_rear_rad_;
    double theta_target_;
    Vector2d pos_target_;
    int state_;
    Stopwatch move_timer_;
    Vector3d start_pose_;
};

#endif // SIMPLEGOALDRIVER_H
