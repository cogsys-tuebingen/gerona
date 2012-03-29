/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/15/2012
   @file   FixedDriver.cpp

*/ 
#include "ramaxxbase/RamaxxMsg.h"
#include "Line2d.h"
#include "MathHelper.h"
#include "FixedDriver.h"
#include "MotionControlNode.h"
FixedDriver::FixedDriver(ros::Publisher &cmd_pub, MotionControlNode *node)
  :node_(node),cmd_pub_(cmd_pub),state_(MotionResult::MOTION_STATUS_STOP)
{
  cmd_v_ = 0;
  cmd_front_rad_ =0.0;
  cmd_rear_rad_= 0.0;
  beta_=0.0;
  dist_measure_threshold_=0.1;
}


void FixedDriver::configure()
{

}


void FixedDriver::setGoal(const motion_control::MotionGoal &goal)
{
  mode_=goal.mode;
  cmd_v_=goal.v;
  cmd_front_rad_=goal.deltaf;
  cmd_rear_rad_=goal.deltar;
  beta_=atan(0.5*(tan(cmd_front_rad_)+tan(cmd_rear_rad_)));
  node_->getWorldPose(start_pose_);
  last_pose_=start_pose_;
  driven_dist_=0.0;
  state_=MotionResult::MOTION_STATUS_MOVING;
  move_timer_.restart();
  laser_scan_.ranges.clear();
}


void FixedDriver::start()
{


}


void FixedDriver::stop()
{

}


int FixedDriver::execute(MotionFeedback &fb, MotionResult &result)
{
  // check collision

  switch (state_) {
  case MotionResult::MOTION_STATUS_COLLISION:
    cmd_v_=0;
    state_=  MotionResult::MOTION_STATUS_STOP;

    break;
  case MotionResult::MOTION_STATUS_SUCCESS:
    state_=MotionResult::MOTION_STATUS_STOP;
    cmd_v_=0;
    break;
  case MotionResult::MOTION_STATUS_STOP:
    cmd_v_=0;
    result.status=MotionResult::MOTION_STATUS_STOP;
    break;
  case MotionResult::MOTION_STATUS_MOVING:
  default:
  {
    bool colliding=checkCollision(beta_,0.3);
    if (colliding) {
      result.status=MotionResult::MOTION_STATUS_COLLISION;
      cmd_v_=0.0;
    } else {
      result.status=MotionResult::MOTION_STATUS_MOVING;
    }
    Vector3d pose;
    node_->getWorldPose(pose);
    double delta_dist=(pose.head<2>()-last_pose_.head<2>()).norm();
    if (delta_dist>dist_measure_threshold_) {
      driven_dist_+=delta_dist;
      last_pose_=pose;
    }
    // return driven distance in feedback
    fb.dist_driven=driven_dist_;
    break;
  }
  }
  ramaxxbase::RamaxxMsg cmd;
  cmd.data.resize(3);
  cmd.data[0].key=ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;
  cmd.data[1].key=ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG;
  if (mode_==motion_control::MotionGoal::MOTION_DIRECT_SPEED) {
    cmd.data[2].key=ramaxxbase::RamaxxMsg::CMD_SPEED_SERVO;
  } else {
    cmd.data[2].key=ramaxxbase::RamaxxMsg::CMD_SPEED;
  }
  cmd.data[0].value=cmd_front_rad_*180.0/M_PI;
  cmd.data[1].value=cmd_rear_rad_*180.0/M_PI;
  cmd.data[2].value=cmd_v_;

  cmd_pub_.publish(cmd);
  return result.status;
}
