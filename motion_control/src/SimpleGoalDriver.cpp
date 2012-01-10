#include <geometry_msgs/Quaternion.h>
#include "ramaxxbase/RamaxxMsg.h"
#include "Line2d.h"
#include "MathHelper.h"
#include "SimpleGoalDriver.h"



SimpleGoalDriver::SimpleGoalDriver(ros::Publisher& cmd_pub,ros::NodeHandle& node)
  :cmd_pub_(cmd_pub),state_(MotionResult::MOTION_STATUS_STOP)
{
  ctrl_.reset();
  cmd_v_ = 0;
  cmd_front_rad_ =0.0;
  cmd_rear_rad_= 0.0;
  configure(node);

}


void SimpleGoalDriver::configure(ros::NodeHandle &node)
{
  double Ta,Kp,e_max,delta_max_deg,default_v;
  // configure dual pid control
  node.param<double>("rowdetect/dualpid/Tt",Tt_,0.1); // system deadtime
  node.param<double>("rowdetect/dualpid/Ta",Ta,0.05); // controller sampling time
  node.param<double>("rowdetect/dualpid/Kp",Kp,0.5);
  node.param<double>("rowdetect/dualpid/e_max",e_max,0.1);
  node.param<double>("rowdetect/dualpid/L",L_,0.38);
  node.param<double>("rowdetect/dualpid/default_v",default_v,0.5);
  node.param<double>("rowdetect/dualpid/delta_max_deg",delta_max_deg,22.0);

  if (delta_max_deg<=0.0) {
    ROS_ERROR("invalid max steer angle delta_max in config. Setting to 20deg");
    delta_max_deg=20.0;
  }

  double delta_max_rad = delta_max_deg*M_PI/180.0;
  ctrl_.configure(Kp,delta_max_rad,e_max,default_v,Ta);
  cmd_v_=0;
}


void SimpleGoalDriver::setGoal(const motion_control::MotionGoal &goal)
{
  goal_pose_global_.pose.position.x=goal.x;
  goal_pose_global_.pose.position.y=goal.y;
  goal_pose_global_.pose.position.z=0;
  goal_pose_global_.pose.orientation=
    tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,goal.theta);
  default_v_ = goal.v;
  if (fabs(default_v_)<0.01) {
    ROS_ERROR("motion_planner:simplegoaldriver robot speed set to %fm/sec - speed too slow",default_v_);
    state_=MotionResult::MOTION_STATUS_MOVE_FAIL;
    return;
  }
  if (goal.mode==motion_control::MotionGoal::MOTION_FOLLOW_TARGET) {
    goal_v_=goal.target_v;
  } else {
    goal_v_=0.0;
  }
  ROS_INFO("simplegoaldriver::setgoal: x=%f y=%f v=%f",goal_pose_global_.pose.position.x,
           goal_pose_global_.pose.position.y,goal_v_);
  start();
}


void SimpleGoalDriver::stop()
{
  cmd_v_=0;
  publish();
}


void SimpleGoalDriver::predictPose(double dt, double deltaf, double deltar, double v,
                                 Vector2d& front_pred, Vector2d& rear_pred)
{

  double beta=atan(0.5*(tan(deltaf)+tan(deltar)));
  double ds = v*dt;
  double dtheta=ds*cos(beta)*(tan(deltaf)-tan(deltar))/L_;
  double thetan=dtheta;
  double yn=ds*sin(dtheta*0.5+beta*0.5);
  double xn=ds*cos(dtheta*0.5+beta*0.5);

  front_pred[0]=xn+cos(thetan)*L_/2.0;
  front_pred[1]=yn+sin(thetan)*L_/2.0;
  rear_pred[0]=xn-cos(thetan)*L_/2.0;
  rear_pred[1]=yn-sin(thetan)*L_/2.0;

}


void SimpleGoalDriver::start()
{
  move_timer_.restart();
  getSlamPose(start_pose_);
  ctrl_.reset();
  state_=MotionResult::MOTION_STATUS_MOVING;
}


int SimpleGoalDriver::driveToGoal(const Vector3d& goal, motion_control::MotionFeedback& feedback,
                                  motion_control::MotionResult& result)
{

  Vector2d front_pred,rear_pred;
  double direction;
  if (goal.x()<0) {
      direction = -1.0;
  } else {
      direction = +1.0;
  }
  predictPose(Tt_,cmd_front_rad_,cmd_rear_rad_,direction*default_v_,
                front_pred, rear_pred);

  Line2d target_line;
  Vector2d target_pos( goal[0], goal[1] );
  target_line.FromAngle( target_pos, goal[2] );
  double ef =target_line.GetSignedDistance(front_pred);
  double er =target_line.GetSignedDistance(rear_pred);
  double deltaf,deltar;
  bool controlled=ctrl_.execute(ef,er,deltaf,deltar);
  if (controlled) {
    cmd_v_=direction*default_v_;
    cmd_front_rad_=-1.0*direction*deltaf;
    cmd_rear_rad_=-1.0*direction*deltar;
  } else {
  }
  // estimate course
  double beta=atan(0.5*(tan(deltaf)+tan(deltar)));
  // add pi to direction if driving backwards
  beta+=direction<0?M_PI:0.0;
  beta=MathHelper::NormalizeAngle(beta);
    // check collision
  bool colliding=checkCollision(beta,0.3);
  if (colliding) {
    result.status=MotionResult::MOTION_STATUS_COLLISION;
    cmd_v_=0.0;
    return MotionResult::MOTION_STATUS_COLLISION;
  } else {
    cmd_v_=direction*default_v_;
    result.status=MotionResult::MOTION_STATUS_MOVING;
    return MotionResult::MOTION_STATUS_MOVING;
  }
}


int SimpleGoalDriver::execute(motion_control::MotionFeedback& feedback,
                         motion_control::MotionResult& result)
{
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
  {
    // transform goal point into local cs
    geometry_msgs::PoseStamped goal_pose_local;

    Vector3d robot_pose;
    goal_pose_global_.header.stamp=ros::Time::now();
    try {
      goal_pose_global_.header.frame_id="/map";
      pose_listener_.transformPose("/base_link",ros::Time(0),goal_pose_global_,"/map",goal_pose_local);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("error with transform goal pose: %s", ex.what());
      return MotionResult::MOTION_STATUS_MOVING;
    }

    // check distance to goal
    Vector3d goal_vec;
    goal_vec.x()=goal_pose_local.pose.position.x;
    goal_vec.y()=goal_pose_local.pose.position.y;
    // goal in reach for 4ws driver?
    double dist_goal=goal_vec.head<2>().norm();
    if ((dist_goal<0.3) ) {
      // goal reached
      result.status=MotionResult::MOTION_STATUS_SUCCESS;
      state_=MotionResult::MOTION_STATUS_SUCCESS;
      cmd_v_=goal_v_;
      ROS_INFO("simplegoaldriver: goal reached");
    } else {
      // driver drives towards goal
      goal_vec.z()=tf::getYaw(goal_pose_local.pose.orientation);
      state_=driveToGoal(goal_vec,feedback,result);

    }
    feedback.dist_goal=dist_goal;
    break;
  }
  default:
    // unknown state
    state_ = MotionResult::MOTION_STATUS_STOP;
    cmd_v_=0;
    result.status=MotionResult::MOTION_STATUS_INTERNAL_ERROR;
    break;
  }
  publish();
  return state_;
}



void SimpleGoalDriver::publish()
{
  ramaxxbase::RamaxxMsg cmd;
  cmd.data.resize(3);
  cmd.data[0].key=ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;
  cmd.data[1].key=ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG;
  cmd.data[2].key=ramaxxbase::RamaxxMsg::CMD_SPEED;
  cmd.data[0].value=cmd_front_rad_*180.0/M_PI;
  cmd.data[1].value=cmd_rear_rad_*180.0/M_PI;
  cmd.data[2].value=cmd_v_;

  cmd_pub_.publish(cmd);
}

