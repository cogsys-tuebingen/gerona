/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/24/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#include "MoveAction.h"

MoveAction::MoveAction (ConfigFileReader *config, const string &config_key, CalibBotInterface *proxy)
  :RobotAction(proxy)
{
  steer_rad_front_=config->GetDouble(config_key+"::steerDegFront",0.0)*M_PI/180.0;
  steer_rad_rear_=config->GetDouble(config_key+"::steerDegRear",0.0)*M_PI/180.0;
  speed_ms_ = config->GetDouble(config_key+"::speed",0.4);
  double steer_ctrl_kp=config->GetDouble(config_key+"::steerCtrlKp",400);
  double steer_ctrl_ki=config->GetDouble(config_key+"::steerCtrlKi",600);
  double threshold=config->GetDouble(config_key+"::ctrlThresholdDeg",10)*M_PI/180.0;

  // configure the steering angle controllers
  ctrl_front_.setPosition(0);
  ctrl_front_.setLimits(1400,3100);
  ctrl_front_.setPiParameters(steer_ctrl_kp,steer_ctrl_ki);
  ctrl_front_.setThreshold(threshold);
  ctrl_front_.setTargetAngle(steer_rad_front_);
  ctrl_rear_.setPosition(1);
  ctrl_rear_.setLimits(1400,3100);
  ctrl_rear_.setPiParameters(steer_ctrl_kp,steer_ctrl_ki);
  ctrl_rear_.setThreshold(threshold);
  ctrl_rear_.setTargetAngle(steer_rad_rear_);
}


void MoveAction::getResults(const string &key, ConfigFileReader &results)
{
  results.SetDouble(key+"::01SlamX",slam_pose_.x());
  results.SetDouble(key+"::02SlamY",slam_pose_.y());
  results.SetDouble(key+"::03SlamTheta",slam_pose_.z());
  results.SetDouble(key+"::04OdoX",odo_pose_.x());
  results.SetDouble(key+"::05OdoY",odo_pose_.y());
  results.SetDouble(key+"::06OdoTheta",odo_pose_.z());
}

