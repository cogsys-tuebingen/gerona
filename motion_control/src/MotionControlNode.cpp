#include <ramaxxbase/RamaxxMsg.h>
#include "CalibDriver.h"
#include "SimpleGoalDriver.h"
#include "FixedDriver.h"
#include "MotionController.h"
#include "MotionControlNode.h"
#include "PatternDriver.h"
#include "PathDriver.h"

using namespace motion_control;

MotionControlNode::MotionControlNode(ros::NodeHandle& nh, const std::string& name)
  :nh_(nh),action_server_(nh,name, false), action_name_(name)
{
  action_server_.registerGoalCallback(boost::bind(&MotionControlNode::goalCallback,this));
  action_server_.registerPreemptCallback(boost::bind(&MotionControlNode::preemptCallback,this));
  action_server_.start();

  nh_.param<string>("world_frame",world_frame_,"/map");
  nh_.param<string>("robot_frame",robot_frame_,"/base_link");
  cmd_ramaxx_pub_ = nh_.advertise<ramaxxbase::RamaxxMsg>
      ("/ramaxx_cmd", 10 );
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>( "/scan", 1, boost::bind(&MotionControlNode::laserCallback, this, _1 ));

  active_ctrl_ = NULL;
  calib_driver_ = new CalibDriver (cmd_ramaxx_pub_, this);
  simple_goal_driver_ = new SimpleGoalDriver (cmd_ramaxx_pub_,this);
  fixed_driver_ =new FixedDriver(cmd_ramaxx_pub_, this);
  pattern_driver_=new PatternDriver(cmd_ramaxx_pub_, this);
  path_driver_ = new PathDriver( cmd_ramaxx_pub_, this );

  action_server_.start();
}


MotionControlNode::~MotionControlNode()
{

  delete calib_driver_;
  delete simple_goal_driver_;
  delete fixed_driver_;
  delete path_driver_;
}


bool MotionControlNode::transformToLocal(const geometry_msgs::PoseStamped &global, Vector3d &local)
{
  geometry_msgs::PoseStamped local_pose;
  bool status=transformToLocal(global,local_pose);
  if (!status) {
    local.x()=local.y()=local.z()=0.0;
    return false;
  }
  local.x()=local_pose.pose.position.x;
  local.y()=local_pose.pose.position.y;
  local.z()=tf::getYaw(local_pose.pose.orientation);
  return true;
}


bool MotionControlNode::transformToLocal(const geometry_msgs::PoseStamped &global_org, geometry_msgs::PoseStamped &local)
{
  geometry_msgs::PoseStamped global(global_org);
  try {
    global.header.frame_id=world_frame_;
    pose_listener_.transformPose(robot_frame_,ros::Time(0),global,world_frame_,local);
    return true;

  } catch (tf::TransformException& ex) {
    ROS_ERROR("error with transform goal pose: %s", ex.what());
    return false;
  }
}


bool MotionControlNode::getWorldPose( Vector3d &pose ) const
{
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg;

    std::string source_frame("base_link");
    std::string target_frame("map");
    try {
      pose_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
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


void MotionControlNode::goalCallback()
{  
  boost::shared_ptr<const motion_control::MotionGoal_<std::allocator<void> > >
    goalptr = action_server_.acceptNewGoal();
  if (active_ctrl_!=NULL && goalptr->mode!=active_ctrl_->getType()) {
    active_ctrl_->stop();
  }
  switch (goalptr->mode) {
    case motion_control::MotionGoal::MOTION_DRIVE_PATTERN:
      active_ctrl_=pattern_driver_;
      active_ctrl_->setGoal(*goalptr);
      break;
    case motion_control::MotionGoal::MOTION_DIRECT_SPEED:
    case motion_control::MotionGoal::MOTION_FIXED_PARAMS:
      active_ctrl_=fixed_driver_;
      active_ctrl_->setGoal(*goalptr);
      break;
    case motion_control::MotionGoal::MOTION_ODO_CALIB:
      active_ctrl_=calib_driver_;
      active_ctrl_->setGoal(*goalptr);
      break;
    case motion_control::MotionGoal::MOTION_FOLLOW_PATH:
      active_ctrl_ = path_driver_;
      active_ctrl_->setGoal( *goalptr );
      break;
    case motion_control::MotionGoal::MOTION_TO_GOAL:
    case motion_control::MotionGoal::MOTION_FOLLOW_TARGET:
      active_ctrl_=simple_goal_driver_;
      active_ctrl_->setGoal(*goalptr);
      break;      
    default:
      ROS_WARN("Motioncontrol invalid motion mode %d requested",goalptr->mode);
      MotionResult result;
      result.status=MotionResult::MOTION_STATUS_INTERNAL_ERROR;
      action_server_.setAborted(result);
      active_ctrl_=0;
  }


}

void MotionControlNode::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (active_ctrl_!=NULL) {
    active_ctrl_->laserCallback(scan);
  }
}


void MotionControlNode::preemptCallback()
{
    if ( active_ctrl_ != NULL ) {
        active_ctrl_->stop(); /// @todo think about this
    }

}


void MotionControlNode::update()
{
  if (active_ctrl_!=NULL) {
    MotionFeedback feedback;
    MotionResult result;
    int status=active_ctrl_->execute(feedback,result);
    switch (status) {
    case MotionResult::MOTION_STATUS_STOP:
      // nothing to do
      break;
    case MotionResult::MOTION_STATUS_MOVING:
      action_server_.publishFeedback(feedback);
      break;
    case MotionResult::MOTION_STATUS_SUCCESS:
        action_server_.setSucceeded( result );
        //ROS_INFO("motioncontrolnode: MOTION_STATUS_SUCCESS");
        break;
    case MotionResult::MOTION_STATUS_COLLISION:
      ROS_INFO("motioncontrolnode: MOTION_STATUS_COLLISION");
    case MotionResult::MOTION_STATUS_INTERNAL_ERROR:
    case MotionResult::MOTION_STATUS_SLAM_FAIL:
    default:
        action_server_.setAborted(result);
        active_ctrl_ = NULL;
        status=MotionResult::MOTION_STATUS_STOP;
        break;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"");
  //ros::NodeHandle nh("~");
  ros::NodeHandle nh;
  MotionControlNode node(nh,"motion_control");

  ros::Rate rate(50);

  while( ros::ok() )
  {
    ros::spinOnce();
    node.update();
    rate.sleep();
  }

  return 0;
}
