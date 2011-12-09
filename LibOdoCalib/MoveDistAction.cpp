/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/16/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/
#include <fstream>
#include <iostream>
#include "ConfigFileReader.h"
#include "MoveDistAction.h"
#include "Calibration.h"

MoveDistAction::MoveDistAction(ConfigFileReader *config, const string& config_key,
                               CalibBotInterface *proxy)
  :MoveAction(config, config_key,proxy)
{

  // configure action
  target_dist_=config->GetDouble(config_key+"::distance",-1);
  cout << "configuring movedistance="<<target_dist_<< " key="<<config_key<< endl;
  if (target_dist_<0.0) {
    cout << "Missing config option "<<config_key+"::distance" << endl;
    exit(-1);
  }


}


MoveDistAction::~MoveDistAction()
{
  // nothing to do
}


void MoveDistAction::getResults(const string &key, ConfigFileReader &results)
{
  MoveAction::getResults(key,results);

}


bool MoveDistAction::execute()
{
  // prepare log
  if (!log_stream_) {
    cout << "Missing logfile stream "<<endl;
    return false;
  }
  // matlab style comment at beginning
  *log_stream_<< "% MoveDistaction steerfrontdeg="<<steer_rad_front_*180.0/M_PI
                 << " steerreardeg="<<steer_rad_rear_*180.0/M_PI
                 <<" dist="<< target_dist_ <<endl;

  *log_stream_<< "%time slamx slamy slamtheta odox odoy odotheta targetdeltafront targetdeltarear halldeltafront halldeltarear servofront servorear ododist odoticksleft odoticksright"<<std::endl;

  proxy_->Read();
  //
  Stopwatch ctrl_timer;
  ctrl_timer.restart();
  double start_dist = proxy_->GetEncoderDistance();
  Vector3d pose;
  proxy_->GetSlamPose(pose);
  double delta_dist=0.0;
  while (delta_dist<target_dist_) {
    proxy_->Read();
    // check collision
    if (checkCollision(0,0.5)) {
      proxy_->SetSpeedSteer(0.0,steer_rad_front_,steer_rad_rear_);
      return false;
    }

    // log data
    proxy_->GetSlamPose(slam_pose_);
    proxy_->GetOdometryPose(odo_pose_);
    DVector hall_volts(4);
    proxy_->GetHallVoltages(hall_volts);
    double hall_front_angle=calcHallAngle(POS_FRONT,hall_volts);
    double hall_rear_angle=calcHallAngle(POS_REAR,hall_volts);
    int odo_ticks_left=proxy_->GetLeftEncoderTicks();
    int odo_ticks_right=proxy_->GetRightEncoderTicks();
    double odo_dist = proxy_->GetEncoderDistance();
    *log_stream_<< mission_timer_->msElapsed()<<" "
      << slam_pose_.x()<< " "<<slam_pose_.y()<< " "<< slam_pose_.z()<< " "
      << odo_pose_.x()<< " "<<odo_pose_.y()<< " "<< odo_pose_.z()<< " "
      << steer_rad_front_ << " "<< steer_rad_rear_<<" "
      << hall_front_angle << " "<<hall_rear_angle << " "
      << odo_dist << " " << odo_ticks_left << " " << odo_ticks_right
      << endl;

    int ctrl_ms = ctrl_timer.msElapsed();
    if (ctrl_ms>200) {
        proxy_->SetSpeedSteer(speed_ms_,steer_rad_front_,steer_rad_rear_);
        ctrl_timer.restart();
        Vector3d new_pose;
        proxy_->GetSlamPose(new_pose);
        delta_dist+=(new_pose-pose).norm();
        pose=new_pose;
    }    
    usleep(50000);
  }
  // stop the robot
  proxy_->SetSpeedSteer(0.0,steer_rad_front_,steer_rad_rear_);
  return true;
}

