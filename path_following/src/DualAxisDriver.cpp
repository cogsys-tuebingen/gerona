
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>

#include <tf/transform_listener.h>
// Project
#include "DualAxisDriver.h"
#include "Line2d.h"
#include "MathHelper.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

DualAxisDriver::DualAxisDriver(ros::NodeHandle& node)
  : pose_listener_(0) {
    mState = DRIVE_STRAIGHT;
    ctrl_.reset();
    default_v_ = 0.5;
    default_turn_v_ = 0.5;
    K_v_ = 1.0;
    cmd_v_ = default_v_;
    cmd_front_rad_ =0.0;
    cmd_rear_rad_= 0.0;

    configure(node);
}


void DualAxisDriver::configure(ros::NodeHandle& node)
{
  node.param<std::string>("rowdetect/dualpid/matlabLog",log_fname_,"");
  if (!log_fname_.empty()) {
      log_stream_.open(log_fname_.c_str());
      log_stream_<< "%time slamx slamy slamtheta odox odoy odotheta pathx pathy paththeta beta thetadiff errfront errrear frontrad rearrad v"<< endl;

      mission_timer_.restart();
      pose_listener_=new tf::TransformListener() ;
      ROS_INFO("opening logfile %s",log_fname_.c_str());

  }
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
  default_turn_v_ = default_v_;
}


void DualAxisDriver::GetCmd(double &speed, double &front_rad, double &rear_rad) const
{
  speed = cmd_v_;
  front_rad = cmd_front_rad_;
  rear_rad = cmd_rear_rad_;
}


bool DualAxisDriver::getRobotPose(Vector3d& pose)
{
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped msg;

  std::string source_frame("base_link");
  std::string target_frame("map");
  try {
    pose_listener_->lookupTransform(target_frame, source_frame, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("error with transform robot pose: %s", ex.what());
    return false;
  }
  tf::transformStampedTFToMsg(transform, msg);

  pose.x() = msg.transform.translation.x;
  pose.y() = msg.transform.translation.y;
  pose(2) = tf::getYaw(msg.transform.rotation);
  return true;
}


void DualAxisDriver::Update( const Eigen::Vector3d &target) {

  driveInRow(target);
}











void DualAxisDriver::predictPose(double dt, double deltaf, double deltar, double v,
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

void DualAxisDriver::driveInRow( const Vector3d &target )
{
  Vector2d front_pred,rear_pred;
  double direction;
  if (target.x()<0) {
      direction = -1.0;
  } else {
      direction = +1.0;
  }
  predictPose(Tt_,cmd_front_rad_,cmd_rear_rad_,direction*default_v_*K_v_,
                front_pred, rear_pred);
  /*ROS_INFO("predict pose %f %f deltaf=%fdeg deltar=%fdeg",front_pred.x(),front_pred.y(),
             cmd_front_rad_*180.0/M_PI, cmd_rear_rad_*180.0/M_PI);
*/
  Line2d target_line;
  Vector2d target_pos( target[0], target[1] );
  target_line.FromAngle( target_pos, target[2] );
  double ef =target_line.GetSignedDistance(front_pred);
  double er =target_line.GetSignedDistance(rear_pred);
  double deltaf,deltar;
  bool controlled=ctrl_.execute(ef,er,deltaf,deltar);
  if (controlled) {
    cmd_v_=direction*K_v_*default_v_;
    cmd_front_rad_=-1.0*direction*deltaf;
    cmd_rear_rad_=-1.0*direction*deltar;
  //  ROS_INFO("ef=%f er=%f deltaf=%fgrad deltar=%fgrad",ef,er,deltaf*180.0/M_PI,deltar*180.0/M_PI);
  } else {
      ROS_INFO("uncontrolled");
  }
  if (log_stream_.is_open()) {
    Vector3d pose;
    if (pose_listener_!=NULL) {
      getRobotPose(pose);
    }
    log_stream_ << mission_timer_.msElapsed()<< " " <<pose.x()<< " " <<pose.y()<< " " << pose(2) << " 0 0 0 "
                << target.x() << " "<< target.y() << " "
                << target[2]<< " 0 0 "<<ef << " "<<er << " "<<cmd_front_rad_ << " "
                << cmd_rear_rad_<< cmd_v_<< endl;

  }

}


void DualAxisDriver::CheckSteerRange( double &steer ) {
  const double steerMax = 30.0*M_PI/180.0;
  if ( steer > steerMax )
    steer = steerMax;
  else if ( steer < -steerMax )
    steer = -steerMax;
}
