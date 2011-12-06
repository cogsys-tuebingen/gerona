
#define EIGEN2_SUPPORT
#include "Eigen/Core"
#include <Eigen/Dense>
#include <Eigen/LeastSquares>
#include "ramaxxbase/RamaxxMsg.h"
#include "Misc.h"
#include "CalibDriver.h"

const double MIN_START_MOVE_DIST = 0.3;
const int    START_MOVE_TIMEOUT_MSEC = 5000;
const double MIN_CALIB_DRIVE_DIST=4.0;
const int MAX_BETA_WAIT_TIME_MS=2000;
enum {
  POS_FRONT=0,
  POS_REAR=1
};

CalibDriver::CalibDriver(ros::Publisher& cmd_pub,ros::NodeHandle& node)
  :cmd_pub_(cmd_pub),state_(CALIB_STATE_STOP)
{
  configure(node);
}

void CalibDriver::configure(ros::NodeHandle &node)
{
  node.param<int>("servoMidFront",servo_front_mid_,2250);
  node.param<int>("servoMidRear",servo_rear_mid_,2250);
  node.param<int>("controlSleepMs",ctrl_sleep_ms_,300);
  node.param<double>("tuningA",tuning_a_,0.6);
  double ks11,ks12,ks21,ks22,kp_yaw;
  node.param<double>("ks11",ks11,1.0);
  node.param<double>("ks12",ks12,0.6787);
  node.param<double>("ks21",ks21,1);
  node.param<double>("ks22",ks22,0.6);
  node.param<double>("kpyaw",kp_yaw,1.2);
  Matrix2d ksinv;
  ksinv << ks11,ks12,ks21,ks22;
  dual_axis_calib_.SetKSinv(ksinv);
  double servo_factor = (3000.0-2250.0)/(22.0*M_PI/180.0);
  dual_axis_calib_.SetServoDelta(POS_FRONT,servo_factor);
  dual_axis_calib_.SetServoDelta(POS_REAR,servo_factor);
  dual_axis_calib_.SetKIyaw(0.0);
  dual_axis_calib_.SetKPyaw(kp_yaw);
  dual_axis_calib_.CalcController(1.0,tuning_a_);

    /*

        ctrl_sleep_ = config->GetInt(config_key+"::ctrlSleep",500);
        ctrl_cnt_ = config->GetInt(config_key+"::ctrlCnt",1);
        tuning_a_ = config->GetDouble( config_key+"::tuningA", 1);
        double ki_yaw = config->GetDouble(config_key+"::kiYaw",0);
        double ks11 = config->GetDouble(config_key+"::ks11",1);
        double ks12 = config->GetDouble(config_key+"::ks12",0.25);
        double ks21 = config->GetDouble(config_key+"::ks21",1);
        double ks22 = config->GetDouble(config_key+"::ks22",-0.175);
        double kp_yaw = config->GetDouble(config_key+"::kpYaw",1);
        double servo_factor = config->GetDouble(config_key+"::servoFactor",(3000.0-2250.0)/(20.0*M_PI/180.0));
        dual_axis_calib_.SetServoDelta(POS_FRONT,servo_factor);
        dual_axis_calib_.SetServoDelta(POS_REAR,servo_factor);
        dual_axis_calib_.SetKIyaw(ki_yaw);
        dual_axis_calib_.SetKPyaw(kp_yaw);
        direction_ = config->GetInt(config_key+"::direction",POS_FRONT);
        if (direction_==POS_FRONT)
            dual_axis_calib_.CalcController(1.0,tuning_a_);
        else
            dual_axis_calib_.CalcController(-1.0,tuning_a_);



        min_delta_dist_ = config->GetDouble(config_key+"::minDelta",0.03);
        servo_front_fixed_ = config->GetInt(config_key+"::servoFront",-9999);
        servo_rear_fixed_ = config->GetInt(config_key+"::servoRear",-9999);
        servo_front_mid_ = config->GetInt(config_key+"::servoFrontMid",2126);
        servo_rear_mid_ = config->GetInt(config_key+"::servoRearMid",2149);
        servo_front_start_ = config->GetInt(config_key+"::servoFrontStart",2250);
        servo_rear_start_ = config->GetInt(config_key+"::servoRearStart",2250);

        setServoMids(servo_front_mid_,servo_rear_mid_);











          */



}


void CalibDriver::setGoal(const motion_control::MotionGoal &goal)
{
  beta_target_=goal.beta;
  speed_=goal.v;
  theta_target_=goal.theta;
  ROS_INFO("calibration start: beta=%f speed=%f theta=%f",beta_target_*180.0/M_PI,speed_,theta_target_*180.0/M_PI);

  start();
}


void CalibDriver::start()
{
  if (state_==CALIB_STATE_STOP) {
    move_timer_.restart();
    getSlamPose(start_pose_);
    cmd_servof_=servo_front_mid_;
    cmd_servor_=servo_rear_mid_;
    cmd_v_=speed_;
    state_=CALIB_STATE_STARTMOVE;
  }
}


int CalibDriver::doStartMove(MotionFeedback& fb, MotionResult& result)
{
  bool colliding=checkCollision(0,0.3);
  if (colliding) {
    result.status=MotionResult::MOTION_STATUS_COLLISION;
    cmd_v_=0.0;
    return MotionResult::MOTION_STATUS_COLLISION;
    state_=CALIB_STATE_STOP;
  } else {

    Vector3d current_pose;
    if (move_timer_.msElapsed()>10000) {
      ROS_WARN("failed to move robot");
      result.status=MotionResult::MOTION_STATUS_MOVE_FAIL;
      state_=CALIB_STATE_STOP;
      cmd_v_=0;
      return result.status;
    }
    bool has_slam=getSlamPose(current_pose);
    if (has_slam) {
      double dist_driven=(current_pose.head<2>()-start_pose_.head<2>()).norm();
      fb.dist_driven=dist_driven;
      if (dist_driven>0.3) {
        ROS_INFO("switching to calib ctrl");
        ctrl_timer_.restart();
        state_=CALIB_STATE_CTRL;
      }
    }
    cmd_v_=speed_;
    cmd_servof_=servo_front_mid_;
    cmd_servor_=servo_rear_mid_;
    return MotionResult::MOTION_STATUS_MOVING;
  }
}


int CalibDriver::doCtrlDrive(MotionFeedback& fb, MotionResult& result)
{
  Vector3d current_pose;
  bool has_slam=getSlamPose(current_pose);
  bool colliding=checkCollision(0,0.3);
  if (!has_slam) {
    if (colliding) {
      result.status=MotionResult::MOTION_STATUS_COLLISION;
      cmd_v_=0.0;
      state_=CALIB_STATE_STOP;
      return MotionResult::MOTION_STATUS_COLLISION;
    }
    // no slam no collision
    // we just continue to drive and hope for slam
    fb.dist_driven=dist_driven_;
    cmd_v_=speed_;
    return MotionResult::MOTION_STATUS_MOVING;
  }
  dist_driven_=(current_pose.head<2>()-start_pose_.head<2>()).norm();
  if (colliding) {
    if (dist_driven_>MIN_CALIB_DRIVE_DIST) {
      // success
      state_=CALIB_STATE_STOP;
      cmd_v_=0;
      return MotionResult::MOTION_STATUS_SUCCESS;
    } else {
      result.status=MotionResult::MOTION_STATUS_COLLISION;
      cmd_v_=0.0;
      state_=CALIB_STATE_STOP;
      return MotionResult::MOTION_STATUS_COLLISION;
    }
  } else {
    cmd_v_=speed_;


    // estimate beta
    beta_estimator_.update(current_pose);
    int direction=0;
    double beta_is;
    int theta_delta=1;
    bool has_beta=beta_estimator_.calcLsBetaAngle(theta_delta,direction,beta_is);
    // do ctrl
    int ctrl_time_ms=ctrl_timer_.msElapsed();
    if (ctrl_time_ms>MAX_BETA_WAIT_TIME_MS) {
      ROS_WARN("failed to get a valid beta estimation for %f seconds. Stopping",ctrl_time_ms/1000.0);
      cmd_v_=0;
      state_=CALIB_STATE_STOP;
      return MotionResult::MOTION_STATUS_SLAM_FAIL;
    }
    if (!has_beta) {
      // continue driving wait for better beta estimation
      return MotionResult::MOTION_STATUS_MOVING;
    }
    if (ctrl_time_ms>ctrl_sleep_ms_) {

      dual_axis_calib_.Execute(ctrl_time_ms/1000.0,beta_target_,beta_is,theta_target_,current_pose.z(),
                             cmd_servof_,cmd_servor_);
      ROS_INFO("ctrl: betaset=%f betais=%f thetaset=%f thetais=%f servof=%f servor=%f",
               beta_target_,beta_is,theta_target_,current_pose.z(),cmd_servof_,cmd_servor_);
      ctrl_timer_.restart();
    }
    return MotionResult::MOTION_STATUS_MOVING;
  }

}


int CalibDriver::execute(motion_control::MotionFeedback& feedback,
                         motion_control::MotionResult& result)
{
  int status;
  switch(state_) {
  case CALIB_STATE_STOP:
  {
    status=MotionResult::MOTION_STATUS_STOP;
    cmd_v_=0;
    break;
  }
  case CALIB_STATE_STARTMOVE:
  {
    status=doStartMove(feedback,result);
    break;
  }
  case CALIB_STATE_CTRL:
  {
    status=doCtrlDrive(feedback,result);
    break;
  }
  default:
    // unknown state
    status = MotionResult::MOTION_STATUS_INTERNAL_ERROR;
    result.status=MotionResult::MOTION_STATUS_INTERNAL_ERROR;
    break;
  }
  publish();
  return status;
}


  double CalibDriver::calcLsBetaAngle(Vector2dVec& ps,double theta, int direction)
  {

    vector<Vector2d*> ptrs(ps.size());
    for (unsigned int i = 0;i<ps.size();++i) {
      ptrs[i]=&(ps[i]);
    }
    Vector2d res;
    int dep=1;
    Vector2d BA;
    linearRegression(ps.size(),&(ptrs[0]),&res,dep);
    if (res(0)>1000) {
      dep=0;
      linearRegression(ps.size(),&(ptrs[0]),&res,dep);
      BA<<res(0),1;
    } else {
      BA<<1,res(0);
    }
    Vector2d A=ps.front();
    Vector2d B = ps.back();
    double d=BA.norm();
    Vector2d dir;
    dir << cos(theta),sin(theta);
    Vector2d CA = dir*d;
    double cosbeta = BA.dot(CA)/BA.squaredNorm();
    Vector2d C=A+d*dir;
    Matrix3d T;
    T << A,B,C,1,1,1;
    double h=T.determinant()/d;
    double beta = acos(cosbeta);
    if (direction==DIRECTION_FWD) {
      if (h>0) {
        return -1.0*beta;
      } else {
        return 1.0*beta;
      }
    } else {
      if (direction==DIRECTION_BWD) beta=beta-M_PI;
      beta=Misc::normalizeAngle(beta);
      if (h>0) {
        return -1.0*beta;
      } else {
        return beta;
      }
    }

  }



  double CalibDriver::calcBetaAngle(const Vector3d& p1, const Vector3d& p2, int direction)
  {

    Vector2d A=p1.head<2>();
    Vector2d B=p2.head<2>();
    double theta = p1(2);
    //if (direction==MODE_BACKWARD) theta=M_PI-theta;
    Vector2d BA=B-A;
    double d = BA.norm();

    if (fabs(d)<1e-4) {
      // moved distance too small for sensible calculation of beta
      return 0;
    }
    Vector2d dir;
    dir << cos(theta),sin(theta);
    Vector2d CA = dir*d;
    double cosbeta = BA.dot(CA)/BA.squaredNorm();
    Vector2d C=A+d*dir;
    Matrix3d T;
    T << A,B,C,1,1,1;
    double h=T.determinant()/d;
    double beta = acos(cosbeta);
    if (direction==DIRECTION_FWD) {
      if (h>0) {
        return -1.0*beta;
      } else {
        return 1.0*beta;
      }
    } else {
      if (direction==DIRECTION_BWD) beta=beta-M_PI;
      beta=Misc::normalizeAngle(beta);
      if (h>0) {
        return -1.0*beta;
      } else {
        return beta;
      }
    }
  }


  void CalibDriver::stop()
  {
    cmd_v_ =0;
    publish();
  }


  void CalibDriver::publish()
  {
    ramaxxbase::RamaxxMsg cmd;
    cmd.data.resize(3);
    cmd.data[0].key=ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_SERVO;
    cmd.data[1].key=ramaxxbase::RamaxxMsg::CMD_STEER_REAR_SERVO;
    cmd.data[2].key=ramaxxbase::RamaxxMsg::CMD_SPEED;
    cmd.data[0].value=cmd_servof_;
    cmd.data[1].value=cmd_servor_;
    cmd.data[2].value=cmd_v_;
    cmd_pub_.publish(cmd);
  }

