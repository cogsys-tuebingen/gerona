
#define EIGEN2_SUPPORT
#include "Eigen/Core"
#include <Eigen/Dense>
#include <Eigen/LeastSquares>
using namespace Eigen;



#include "Misc.h"
#include "CalibBotInterface.h"
#include "Calibration.h"
#include "RobotAction.h"

    RobotAction::RobotAction(CalibBotInterface *proxy)
      :proxy_(proxy),laser_env_(21)
{

}


bool RobotAction::checkCollision(double course,double threshold)
{
  FVector laserRanges;
  float minAngle,maxAngle;
  proxy_->GetLaserReadings(direction_,laserRanges,minAngle,maxAngle);
  laser_env_.ProcessLaserScan(laserRanges,minAngle,maxAngle);
  bool isColliding=laser_env_.CheckCollision(course,0.50,threshold);
  if (isColliding) {
    cout << "colliding"<< endl;
    return true;
  }
  return false;
}


bool RobotAction::startMoving(double speed_ms, double steer_front_rad, double steer_rear_rad, double min_dist, double max_time_ms)
{
  proxy_->SetSpeedSteer(speed_ms,steer_front_rad,steer_rear_rad);

  int loop_sleep_ms=6;
  int loop_cnt = max_time_ms/loop_sleep_ms;
  proxy_->Read();
  Vector3d start_pose,slam_pose;
  proxy_->GetSlamPose(start_pose);
  double dist_sq=min_dist*min_dist;
  Stopwatch timer;
  timer.restart();
  for (int i=0;i<loop_cnt;++i) {
    proxy_->Read();
    // check collision
    double threshold = 0.6;
    bool isColliding = checkCollision(0,threshold);
    if (isColliding) {
      proxy_->SetSpeedAndServos(0,steer_front_rad,steer_rear_rad);
      cout << "Stopping"<< endl;
      return false;
    }
    proxy_->GetSlamPose(slam_pose);
    Vector3d delta=slam_pose-start_pose;
    if (delta.head<2>().squaredNorm()>dist_sq) {
      return true;
    }
    usleep(loop_sleep_ms*1000);
    if (timer.msElapsed()>1000) {
      cout << "resent speed cmd"<<endl;
      proxy_->SetSpeedSteer(speed_ms,steer_front_rad,steer_rear_rad);
      timer.restart();
    }
  }
  return false;
}




bool RobotAction::startMoving(double speed_ms, int servo_f, int servo_r, double min_dist, double max_time_ms)
{
  proxy_->SetSpeedAndServos(speed_ms,servo_f,servo_r);

  int loop_sleep_ms=6;
  int loop_cnt = max_time_ms/loop_sleep_ms;
  proxy_->Read();
  Vector3d start_pose,slam_pose;
  proxy_->GetSlamPose(start_pose);
  double dist_sq=min_dist*min_dist;
  Stopwatch timer;
  timer.restart();
  for (int i=0;i<loop_cnt;++i) {
    proxy_->Read();
    // check collision
    double threshold = 0.6;
    bool isColliding = checkCollision(0,threshold);
    if (isColliding) {
      proxy_->SetSpeedAndServos(0,servo_f,servo_r);
      cout << "Stopping"<< endl;
      return false;
    }
    proxy_->GetSlamPose(slam_pose);
    Vector3d delta=slam_pose-start_pose;
    if (delta.head<2>().squaredNorm()>dist_sq) {
      return true;
    }
    usleep(loop_sleep_ms*1000);
    if (timer.msElapsed()>1000) {
      cout << "resent speed cmd"<<endl;
      proxy_->SetSpeedAndServos(speed_ms,servo_f,servo_r);
      timer.restart();
    }
  }
  return false;
}


double RobotAction::calcLsBetaAngle(Vector2dVec& ps,double theta, int direction)
{

  vector<Vector2d*> ptrs(ps.size());
  for (int i = 0;i<ps.size();++i) {
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
  if (direction==POS_FRONT) {
    if (h>0) {
      return -1.0*beta;
    } else {
      return 1.0*beta;
    }
  } else {
    if (direction_==POS_REAR) beta=beta-M_PI;
    beta=Misc::normalizeAngle(beta);
    if (h>0) {
      return -1.0*beta;
    } else {
      return beta;
    }
  }

}



double RobotAction::calcBetaAngle(const Vector3d& p1, const Vector3d& p2, int direction)
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
  if (direction==POS_FRONT) {
    if (h>0) {
      return -1.0*beta;
    } else {
      return 1.0*beta;
    }
  } else {
    if (direction_==POS_REAR) beta=beta-M_PI;
    beta=Misc::normalizeAngle(beta);
    if (h>0) {
      return -1.0*beta;
    } else {
      return beta;
    }
  }
}



double RobotAction::calcHallAngle(int position, const DVector& hall_volts)
{
  if (hall_volts.size()<4) {
    cout << "error in calc hall voltages"<<__FILE__ << ":"<<__LINE__<< endl;
    return 0;
  }

  if (position==POS_FRONT) {
    double x1=hall_volts[0];
    double x2=hall_volts[1];
    /*
        double angle1=-3.160683*pow(x1,4)+22.818228*pow(x1,3)+-61.869028*pow(x1,2)+73.991555*x1+-32.412426;

        double angle2=0.833344*pow(x2,4)+-5.190334*pow(x2,3)+12.292570*pow(x2,2)+-12.697293*x2+4.392519;
*/


    /*
// gloin values
        double angle1=0.430669+-0.220365*pow((x1+-0.988286),2)+-0.125045*pow((x1+-0.988286),4);
        double angle2=-0.532563+0.312018*pow((x2+-0.485655),2)+-0.105408*pow((x2+-0.485655),4)+0.029200*pow((x2+-0.485655),6);
*/
    // thrain with tractor tires
    double angle1=0.426079+-0.224702*pow((x1+-1.026541),2)+-0.136907*pow((x1+-1.026541),4);
    double angle2 =-0.404312+0.532972*pow((x2+-1.016608),2)+-0.280574*pow((x2+-1.016608),4)+0.148680*pow((x2+-1.016608),6);


    double angle;
    if (x1<1.9)
      angle=angle1;
    else if (x2<1.9)
      angle=angle2;
    else
      angle=(angle1+angle2)/2.0;
    //cout <<" ahall angle fwd="<<angle*180.0/M_PI << endl;
    return angle;
  } else {
    double x1=hall_volts[2];
    double x2=hall_volts[3];
    /*
// gloin values
        double angle1=-0.378514+0.193966*pow((x1+-0.616192),2)+0.031672*pow((x1+-0.616192),4)+-0.041144*pow((x1+-0.616192),6)+0.013846*pow((x1+-0.616192),8);
        double angle2=0.381402+-0.155638*pow((x2+0.026753),2)+0.093779*pow((x2+0.026753),4)+-0.046377*pow((x2+0.026753),6)+0.009954*pow((x2+0.026753),8)+-0.000804*pow((x2+0.026753),10);
*/
    // thrain with tractor tires
    double angle1=-0.362759+0.311860*pow((x1+-0.760034),2)+-0.118582*pow((x1+-0.760034),4)+0.040773*pow((x1+-0.760034),6)+0.003674*pow((x1+-0.760034),8);
    double angle2=0.412166+-0.126369*pow((x2+0.382263),2)+0.064031*pow((x2+0.382263),4)+-0.023409*pow((x2+0.382263),6)+0.003665*pow((x2+0.382263),8)+-0.000216*pow((x2+0.382263),10);



    double angle;
    // magic numbers taken from plot
    if (x1<1.8)
      angle=angle1;
    else if (x2<1.9)
      angle=angle2;
    else
      angle=(angle1+angle2)/2.0;
    //cout <<" ahall angle rear="<<angle*180.0/M_PI << endl;
    return angle;
  }
}

