#include <ramaxxbase/RamaxxMsg.h>
#include "CalibDriver.h"
#include "SimpleGoalDriver.h"
#include "MotionController.h"
#include "MotionControlNode.h"

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
  calib_driver_ = new CalibDriver (cmd_ramaxx_pub_);
  simple_goal_driver_ = new SimpleGoalDriver (cmd_ramaxx_pub_);
}


MotionControlNode::~MotionControlNode()
{

  delete calib_driver_;
  delete simple_goal_driver_;
}

void MotionControlNode::goalCallback()
{
  boost::shared_ptr<const motion_control::MotionGoal_<std::allocator<void> > >
    goalptr = action_server_.acceptNewGoal();
  if (active_ctrl_!=NULL && goalptr->mode!=active_ctrl_->getType()) {
    active_ctrl_->stop();
  }
  switch (goalptr->mode) {
    case motion_control::MotionGoal::MOTION_ODO_CALIB:
      active_ctrl_=calib_driver_;
      break;
    case motion_control::MotionGoal::MOTION_TO_GOAL:
      active_ctrl_=simple_goal_driver_;
      break;
    default:
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


}


void MotionControlNode::update()
{
  if (active_ctrl_!=NULL) {
    MotionFeedback feedback;
    MotionResult result;
    int status=active_ctrl_->execute(feedback,result);
    switch (status) {
    case MOTION_RUN:
      action_server_.publishFeedback(feedback);
      break;
    case MOTION_DONE:
      if (result.status==MotionResult::MOTION_STATUS_OK)
        action_server_.setSucceeded(result);
      else
        action_server_.setAborted(result);
      break;
    default:
      action_server_.setAborted(result);
      break;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"motion_control");
  ros::NodeHandle nh("~");

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
