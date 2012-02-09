#include <ramaxxbase/RamaxxMsg.h>
#include "CalibDriver.h"
#include "SimpleGoalDriver.h"
#include "FixedDriver.h"
#include "MotionController.h"
#include "MotionControlNode.h"
#include "PatternDriver.h"
#include "RsPathDriver.h"

MotionControlNode::MotionControlNode(ros::NodeHandle& node, const std::string& name)
  :action_server_(node,name, false), action_name_(name)
{
  action_server_.registerGoalCallback(boost::bind(&MotionControlNode::goalCallback,this));
  action_server_.registerPreemptCallback(boost::bind(&MotionControlNode::preemptCallback,this));
  action_server_.start();
  cmd_ramaxx_pub_ = node.advertise<ramaxxbase::RamaxxMsg>
      ("/ramaxx_cmd", 100);
  scan_sub_ = node.subscribe<sensor_msgs::LaserScan>( "/scan", 1, boost::bind(&MotionControlNode::laserCallback, this, _1 ));

  active_ctrl_ = NULL;
  calib_driver_ = new CalibDriver (cmd_ramaxx_pub_, node);
  simple_goal_driver_ = new SimpleGoalDriver (cmd_ramaxx_pub_, node);
  rspath_driver_ = new RsPathDriver(cmd_ramaxx_pub_, node);
  fixed_driver_ =new FixedDriver(cmd_ramaxx_pub_, node);
  pattern_driver_=new PatternDriver(cmd_ramaxx_pub_, node);

  action_server_.start();

  //path_driver_->configure(node);
}


MotionControlNode::~MotionControlNode()
{
  delete rspath_driver_;
  delete calib_driver_;
  delete simple_goal_driver_;
  delete fixed_driver_;
}

void MotionControlNode::goalCallback()
{
  ROS_INFO("motion control: received goal");
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
    case motion_control::MotionGoal::MOTION_TO_GOAL:
    case motion_control::MotionGoal::MOTION_FOLLOW_TARGET:
      active_ctrl_=simple_goal_driver_;
      active_ctrl_->setGoal(*goalptr);
      break;      

    case motion_control::MotionGoal::MOTION_FOLLOW_RSPATH:
      active_ctrl_=rspath_driver_;
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
  ROS_WARN("preempt goal called");

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
        action_server_.setSucceeded(result);
        ROS_INFO("motioncontrolnode: MOTION_STATUS_SUCCESS");
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
