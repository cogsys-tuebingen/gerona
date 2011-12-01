
#define EIGEN2_SUPPORT
#include "Eigen/Core"
#include <Eigen/Dense>
#include <Eigen/LeastSquares>
#include "ramaxxbase/RamaxxMsg.h"
#include "Misc.h"
#include "CalibDriver.h"

const double MIN_START_MOVE_DIST = 0.3;
const int    START_MOVE_TIMEOUT_MSEC = 5000;

CalibDriver::CalibDriver(ros::Publisher& cmd_pub)
  :state_(CALIB_STATE_STOP)
{
}

void CalibDriver::configure(ros::NodeHandle &node)
{
  node.param<int>("servoMidFront",servo_front_mid_,2250);
  node.param<int>("servoMidRear",servo_rear_mid_,2250);
}


void CalibDriver::setGoal(const motion_control::MotionGoal &goal)
{
  beta_target_=goal.beta;
  speed_=goal.v;
  theta_target_=goal.theta;
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

}


int CalibDriver::doCtrlDrive(MotionFeedback& fb, MotionResult& result)
{
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


int CalibDriver::execute(motion_control::MotionFeedback& feedback,
                         motion_control::MotionResult& result)
{
  int status=MOTION_DONE;
  switch(state_) {
  case CALIB_STATE_STOP:
  {
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
    status = MOTION_DONE;
    result.status=MotionResult::MOTION_STATUS_INTERNAL_ERROR;
    break;
  }
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

