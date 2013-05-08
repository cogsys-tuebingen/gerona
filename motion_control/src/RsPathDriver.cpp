/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com

   @author Karsten Bohlmann
   @date   1/24/2012
   @file   RsPathDriver.cpp

*/ 
#include <geometry_msgs/Quaternion.h>

#include "ramaxx_msgs/RamaxxMsg.h"

#include "RsPathDriver.h"

RsPathDriver::RsPathDriver(ros::Publisher& cmd_pub,ros::NodeHandle& nh)
  :node_handle_(nh),cmd_pub_(cmd_pub),state_(MotionResult::MOTION_STATUS_STOP)
{
  ctrl_.reset();
  cmd_v_ = 0;
  cmd_front_rad_ =0.0;
  cmd_rear_rad_= 0.0;
  configure(node_handle_);

}


void RsPathDriver::configure(ros::NodeHandle &node)
{
  double e_max,delta_max_deg;
  // configure dual pid control
  node.param<double>("rowdetect/dualpid/Tt",Tt_,0.0); // system deadtime
  node.param<double>("rowdetect/dualpid/e_max",e_max,0.1);
  node.param<double>("rowdetect/dualpid/L",L_,0.38);
  node.param<double>("rowdetect/dualpid/delta_max_deg",delta_max_deg,22.0);

  if (delta_max_deg<=0.0) {
    ROS_ERROR("invalid max steer angle delta_max in config. Setting to 20deg");
    delta_max_deg=20.0;
  }

  double delta_max_rad = delta_max_deg*M_PI/180.0;
  cmd_v_=0;
}


void RsPathDriver::setGoal(const motion_control::MotionGoal &goal)
{
  // set driving speed
  default_v_ = goal.v;
  if (fabs(default_v_)<0.01) {
    ROS_ERROR("motion_planner:RsPathDriver robot speed set to %fm/sec - speed too slow",default_v_);
    state_=MotionResult::MOTION_STATUS_GOAL_FAIL;
    return;
  }
  goal_v_=goal.target_v;
  pos_tolerance_=goal.pos_tolerance;
  // copy the given path
  goal_path_global_=goal.path;
  path_idx_=0;
  if (!goal.path_topic.empty()) {
    // ***todo
    // ros should automatically unsubscribe previous subscriptions
    // test this
    ROS_INFO( "Subscribing to path topic: %s", goal.path_topic.c_str());
    path_subscriber_=node_handle_.subscribe<nav_msgs::Path>
        (goal.path_topic, 2, boost::bind(&RsPathDriver::updatePath, this, _1));
  }
  if (goal_path_global_.poses.empty() && goal.path_topic.empty()) {
    ROS_ERROR("empty path");
    state_=MotionResult::MOTION_STATUS_STOP;
  } else {
    state_=MotionResult::MOTION_STATUS_MOVING;
  }

  laser_scan_.ranges.clear();
  start();
}


void RsPathDriver::stop()
{
  cmd_v_=0;
  publish();
}


void RsPathDriver::updatePath (const nav_msgs::PathConstPtr& path)
{
  ROS_INFO( "Updating path" );
  goal_path_global_=*path;
  if (goal_path_global_.poses.size()>0) {
    goal_pose_global_=goal_path_global_.poses[0];
    path_idx_=0;
    state_=MotionResult::MOTION_STATUS_MOVING;
  } else {
    ROS_ERROR("empty path");
    state_=MotionResult::MOTION_STATUS_STOP;
  }
}


void RsPathDriver::predictPose(double dt, double deltaf, double deltar, double v,
                               Vector3d& pred_pose)
{

  double beta=atan(0.5*(tan(deltaf)+tan(deltar)));
  double ds = v*dt;
  double dtheta=ds*cos(beta)*(tan(deltaf)-tan(deltar))/L_;
  double thetan=dtheta;
  double yn=ds*sin(dtheta*0.5+beta*0.5);
  double xn=ds*cos(dtheta*0.5+beta*0.5);
  pred_pose.x()=xn;
  pred_pose.y()=yn;
  pred_pose.z()=thetan;
}


void RsPathDriver::calcIcr(const Vector3d &p0, const Vector3d &p1,
                           const Vector3d&p2,Vector2d &m, double &r, double &deltaf)
{
  // calc circle through 3 points
  // http://2000clicks.com/mathhelp/GeometryConicSectionCircleEquationGivenThreePoints.aspx
  double a,b,c,d,e,f;
  a=p0.x();
  b=p0.y();
  c=p1.x();
  d=p1.y();
  e=p2.x();
  f=p2.y();
  m.x()= (0.5)*((a*a+b*b)*(f-d) + (c*c+d*d)*(b-f) + (e*e+f)*(d-b)) / (a*(f-d)+c*(b-f)+e*(d-b));
  m.y()= (0.5)*((a*a+b*b)*(e-c) + (c*c+d*d)*(a-e) + (e*e+f*f)*(c-a)) / (b*(e-c)+d*(a-e)+f*(c-a));

  ROS_INFO("r0x=%f y=%f z=%fdeg r1x=%f r1y=%f z=%fdeg",a,b,p0.z()*180.0/M_PI,
           c,d,p1.z()*180.0/M_PI);

  ROS_INFO("mx=%f my=%f",m.x(),m.y());

  r=(p0.head<2>()-m).norm();
  deltaf=atan(L_/r);
  if (m.y()<0) {
    // right curve
    deltaf=fabs(deltaf)*-1.0;
  } else {
    deltaf=fabs(deltaf);
  }

}


void RsPathDriver::calcIcr(const Vector3d& r0,const Vector3d& r1,
                           Vector2d& m,double& r, double& deltaf)
{

  Vector2d a(r0.x(),r0.y()),c(r1.x(),r1.y());
  Vector2d ba(cos(r0.z()+M_PI/2.0),sin(r0.z()+M_PI/2.0)),
      dc(cos(r1.z()+M_PI/2.0),sin(r1.z()+M_PI/2.0));
  Vector2d b,d;
  b=a+ba;
  d=c+dc;
  double x1,y1,x2,y2,x3,y3,x4,y4;
  x1=a.x();y1=a.y();
  x2=b.x();y2=b.y();
  x3=c.x();y3=c.y();
  x4=d.x();y4=d.y();
  ROS_INFO("r0x=%f y=%f z=%fdeg r1x=%f r1y=%f z=%fdeg",x1,y1,r0.z()*180.0/M_PI,
           x3,y3,r1.z()*180.0/M_PI);
  // wikipedia line-line-intersection
  // intersection line ab with cd
  double nom=(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
  if (fabs(nom)<1e-6) {
    ROS_INFO("nom niedrig=%f",nom);
    deltaf=0;
    return;
  }
  m.x()=((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/(nom);
  m.y()=((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/nom;
  ROS_INFO("mx=%f my=%f",m.x(),m.y());
  r=(a-m).norm();
  deltaf=atan(L_/r);
  if (m.y()<0) {
    // right curve
    deltaf=fabs(deltaf)*-1.0;
  } else {
    deltaf=fabs(deltaf);
  }

}


void RsPathDriver::start()
{
  move_timer_.restart();
  getWorldPose(start_pose_);
}


int RsPathDriver::execute(motion_control::MotionFeedback& feedback,
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
    unsigned path_size=goal_path_global_.poses.size();
    geometry_msgs::PoseStamped  goal_local1, goal_local2;
    while (path_idx_<path_size) {
      try {
        geometry_msgs::PoseStamped  goal_global1=goal_path_global_.poses[path_idx_];
        // transform goal point into local cs
        goal_global1.header.frame_id="/map";
        listener_.transformPose("/base_link",ros::Time(0), goal_global1, "/map",goal_local1);
        Vector2d p1;
        p1.x()=goal_local1.pose.position.x;
        p1.y()=goal_local1.pose.position.y;
        if (p1.norm()<pos_tolerance_) {
          ROS_WARN("waypoint reached num=%d idx=%d x=%f y=%f",path_size,path_idx_,p1.x(),p1.y());
          ++path_idx_;
          continue;
        } else {
          break;
        }
      } catch (tf::TransformException ex){
        ROS_ERROR_STREAM("PathDriver: Cannot transform point -> " << ex.what());
        return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
      }
    }
    if (path_idx_>=path_size) {
      ROS_INFO("goal reached");
      result.status=MotionResult::MOTION_STATUS_SUCCESS;
      state_=MotionResult::MOTION_STATUS_SUCCESS;
      cmd_v_=goal_v_;
    } else {
      Vector3d  pred_local;
      Vector3d p1(goal_local1.pose.position.x,
                  goal_local1.pose.position.y,
                  tf::getYaw(goal_local1.pose.orientation));
      double direction;

      if (p1.x()<0) direction=-1.0;
          else direction=+1.0;
      double r1=0,deltaf;
      predictPose(Tt_,cmd_front_rad_,cmd_rear_rad_,direction*default_v_,pred_local);

      Vector2d m;
      if (path_idx_==path_size-1) {
        calcIcr(pred_local,p1,m,r1,deltaf);
      } else {
        try {

          geometry_msgs::PoseStamped  goal_global2=goal_path_global_.poses[path_idx_+1];
          // transform goal point into local cs
          goal_global2.header.frame_id="/map";
          listener_.transformPose("/base_link",ros::Time(0), goal_global2, "/map",goal_local2);
          Vector3d p2;
          p2.x()=goal_local2.pose.position.x;
          p2.y()=goal_local2.pose.position.y;
          p2.z()=tf::getYaw(goal_local2.pose.orientation);
          calcIcr(pred_local,p1,p2,m,r1,deltaf);

        } catch (tf::TransformException ex){
          ROS_ERROR_STREAM("PathDriver: Cannot transform point -> " << ex.what());
          return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
        }

      }
      ROS_INFO("predicted r=%f deltaf=%fdeg",r1,deltaf*180.0/M_PI);
      cmd_v_=default_v_;
      cmd_front_rad_=deltaf;
      cmd_rear_rad_=0.0;
    }

    feedback.dist_goal=0.0;
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


void RsPathDriver::publish()
{
  ramaxx_msgs::RamaxxMsg cmd;
  cmd.data.resize(3);
  cmd.data[0].key=ramaxx_msgs::RamaxxMsg::CMD_STEER_FRONT_DEG;
  cmd.data[1].key=ramaxx_msgs::RamaxxMsg::CMD_STEER_REAR_DEG;
  cmd.data[2].key=ramaxx_msgs::RamaxxMsg::CMD_SPEED;
  cmd.data[0].value=cmd_front_rad_*180.0/M_PI;
  cmd.data[1].value=cmd_rear_rad_*180.0/M_PI;
  cmd.data[2].value=cmd_v_;

  cmd_pub_.publish(cmd);
}


