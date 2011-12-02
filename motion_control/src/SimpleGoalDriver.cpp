#include "SimpleGoalDriver.h"



SimpleGoalDriver::SimpleGoalDriver(ros::Publisher& cmd_pub)
  :state_(MOTION_DONE)
{
  ctrl_.reset();
  default_v_ = 0.5;
  K_v_ = 1.0;
  cmd_v_ = 0;
  cmd_front_rad_ =0.0;
  cmd_rear_rad_= 0.0;


}


void SimpleGoalDriver::configure(ros::NodeHandle &node)
{
  double Ta,Kp,e_max,delta_max_deg;
  // configure dual pid control
  node.param<double>("rowdetect/dualpid/v",default_v_,0.7);
  node.param<double>("rowdetect/dualpid/v",default_turn_v_,0.5);
  node.param<double>("rowdetect/dualpid/Tt",Tt_,0.1); // system deadtime
  node.param<double>("rowdetect/dualpid/Ta",Ta,0.05); // controller sampling time
  node.param<double>("rowdetect/dualpid/Kp",Kp,0.5);
  node.param<double>("rowdetect/dualpid/e_max",e_max,0.1);
  node.param<double>("rowdetect/dualpid/L",L_,0.38);
  node.param<double>("rowdetect/dualpid/delta_max_deg",delta_max_deg,22.0);

  if (delta_max_deg<=0.0) {
    ROS_ERROR("invalid max steer angle delta_max in config. Setting to 20deg");
    delta_max_deg=20.0;
  }
  double delta_max_rad = delta_max_deg*M_PI/180.0;
  ctrl_.configure(Kp,delta_max_rad,e_max,default_v_,Ta);
  cmd_v_=0;
}


void SimpleGoalDriver::setGoal(const motion_control::MotionGoal &goal)
{
  pos_target_.x()=goal.x;
  pos_target_.y()=goal.y;
  default_v_=goal.v;
  theta_target_=goal.theta;
}


void SimpleGoalDriver::start()
{
  if (state_==MOTION_DONE) {
    move_timer_.restart();
    getSlamPose(start_pose_);
    state_=MOTION_RUN;
  }
}


  bool colliding=checkCollision(0,0.3);
  if (colliding) {
    result.status=MotionResult::MOTION_STATUS_COLLISION;
    cmd_v_=0.0;
    return MOTION_DONE;
  } else {
    cmd_v_=speed_;
    Vector3d pose;
    getSlamPose(pose);
    double dist=(pose.head<2>()-start_pose_.head<2>()).norm();
    if (dist>MIN_START_MOVE_DIST) {
      state_=CALIB_STATE_CTRL;
    }
    return MOTION_RUN;
  }
}


int SimpleGoalDriver::execute(motion_control::MotionFeedback& feedback,
                         motion_control::MotionResult& result)
{
  switch (state_) {
  case MOTION_DONE:
  {
    cmd_v_=0;
    break;
  }
  case MOTION_RUN:
  {
    break;
  }
  default:
    // unknown state
    status = MOTION_DONE;
    result.status=MotionResult::MOTION_STATUS_INTERNAL_ERROR;
    break;
  }
  return status;
}
