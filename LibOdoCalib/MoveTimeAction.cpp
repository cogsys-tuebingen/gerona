/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/22/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/
#include <fstream>
#include <iostream>
#include "MoveTimeAction.h"
#include "Calibration.h"


MoveTimeAction::MoveTimeAction(ConfigFileReader *config, const string& config_key,
                               CalibBotInterface *proxy)
  :MoveAction(config,config_key,proxy)
{
  // configure action
  move_time_msec_ = config->GetDouble(config_key+"::time",-1.0)*1000;
  do_steer_ctrl_ = config->GetBool(config_key+"::steerCtrl",true);
  cout << "configuring movetime="<<move_time_msec_<< "ms key="<<config_key<< endl;
  if (move_time_msec_<0) {
    cout << "Missing config option "<<config_key+"::time" << endl;
    exit(-1);
  }

}


MoveTimeAction::~MoveTimeAction()
{
  // nothing to do
}

void MoveTimeAction::getResults(const string &key, ConfigFileReader &results)
{
  MoveAction::getResults(key,results);
}


bool MoveTimeAction::execute()
{
  // prepare log
  if (!log_stream_) {
    cout << "Missing logfile stream "<<endl;
    return false;
  }
  // matlab style comment at beginning
  *log_stream_<< "% Movetimeaction steerfrontdeg="<<steer_rad_front_*180.0/M_PI
                 << " steerreardeg="<<steer_rad_rear_*180.0/M_PI
                 <<" timemsec="<<move_time_msec_ <<endl;

  *log_stream_<< "%time slamx slamy slamtheta odox odoy odotheta targetdeltafront targetdeltarear halldeltafront halldeltarear servofront servorear ododist odoticksleft odoticksright"<<endl;
  proxy_->Read();
  //
  int servo_front_val = ctrl_front_.getStaticServoValue(steer_rad_front_);
  int servo_rear_val = ctrl_rear_.getStaticServoValue(steer_rad_rear_);
  Stopwatch ctrl_timer;
  Stopwatch move_timer;
  move_timer.restart();
  //proxy_->SetSpeedAndServos(speed_ms_,servo_front_val,servo_rear_val);
  proxy_->SetSpeedSteer(speed_ms_,steer_rad_front_,steer_rad_rear_);
  // sleep 600msec to give the servos time to adjust
  //usleep(800000);
  cout << "movetime servofront="<<servo_front_val << " rear="<<servo_rear_val<< "speed="<<speed_ms_<< endl;
  ctrl_timer.restart();
  while (move_timer.msElapsed()<move_time_msec_) {
    proxy_->Read();
    // check collision
    if (checkCollision(0,0.5)) {
      proxy_->SetSpeedAndServos(0.0,servo_front_val,servo_rear_val);      
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
      << hall_front_angle << " "<<hall_rear_angle<< " "
      << servo_front_val << " " << servo_rear_val << " "
      << odo_dist << " " << odo_ticks_left << " " << odo_ticks_right
      << endl;
    int ctrl_ms = ctrl_timer.msElapsed();
    if (ctrl_ms>300) {
        if (do_steer_ctrl_) {
          ctrl_front_.execute(ctrl_ms/1000.0,hall_front_angle,servo_front_val);
          ctrl_rear_.execute(ctrl_ms/1000.0,hall_rear_angle,servo_rear_val);
        }
        proxy_->SetSpeedAndServos(speed_ms_,servo_front_val,servo_rear_val);
        ctrl_timer.restart();
    }




    usleep(20000);
  }
  // stop the robot
  proxy_->SetSpeedAndServos(0.0,servo_front_val,servo_rear_val);
  // read and save end positions
  proxy_->Read();
  proxy_->GetSlamPose(slam_pose_);
  proxy_->GetOdometryPose(odo_pose_);
  return true;
}
