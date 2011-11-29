#include <ramaxxbase/RamaxxMsg.h>
#include "CalibDriver.h"

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
  drive_calib_ = new CalibDriver (cmd_ramaxx_pub_);
  //drive_to_goal_ = new DriveToGoal ();
}


MotionControlNode::~MotionControlNode()
{

  delete drive_calib_;
  //delete drive_to_goal_;
}

void MotionControlNode::goalCallback()
{
  boost::shared_ptr<const motion_control::MotionGoal_<std::allocator<void> > >
    goal = action_server_.acceptNewGoal();
  if (active_ctrl_!=NULL && goal->mode!=active_ctrl_->getType()) {
    active_ctrl_->stop();
  }
  switch (goal->mode) {
    case motion_control::MotionGoal::MOTION_ODO_CALIB:
      active_ctrl_=drive_calib_;

  }


}

void MotionControlNode::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (active_ctrl_!=NULL) {
    active_ctrl_->laserCallBack(scan);
  }
}


void MotionControlNode::preemptCallback()
{


}


void MotionControlNode::update()
{
  if (active_ctrl_!=NULL) {
    active_ctrl_->execute();
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
