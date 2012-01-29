#include <visualization_msgs/Marker.h>

#include "FollowTestNode.h"
#define RTOD(r)   ((r) * 180 / M_PI)
const double DEFAULT_V = 0.6;

FollowTestNode::FollowTestNode(ros::NodeHandle& nh)
  : action_client_("motion_control"),n_(nh), state_(STATE_S_START), default_v_(DEFAULT_V),
    has_path_(false)
{



  path_subscriber_ = nh.subscribe<nav_msgs::Path>
      ("/rs_path", 1, boost::bind(&FollowTestNode::pathReceived, this, _1));


  ROS_INFO("sampling inited");
  state_=STATE_S_WAIT_PATH;

}


FollowTestNode::~FollowTestNode()
{
  // nothing to do
}



void FollowTestNode::pathReceived(const nav_msgs::PathConstPtr &path)
{
  if (state_==STATE_S_WAIT_PATH) {
    ROS_INFO("pathfollower: path received with %zu poses",path->poses.size());
    if (path->poses.size()>0) {
      motion_control::MotionGoal goal;
      goal.mode=motion_control::MotionGoal::MOTION_FOLLOW_PATH;
      goal.pos_tolerance=0.1;
      goal.v=0.5;
      goal.target_v=0;
      goal.path=*path;
      goal.path_topic="";
      action_client_.sendGoal(goal,boost::bind(&FollowTestNode::doneCallback,this,_1,_2),
                              boost::bind(&FollowTestNode::activeCallback,this),
                              boost::bind(&FollowTestNode::feedbackCallback,this,_1));
      state_=STATE_S_DRIVE_PATH;
    } else {
      ROS_ERROR("no good path received wait for new circle");
      state_=STATE_S_WAIT_PATH;
    }

  }
}


void FollowTestNode::doneCallback(const actionlib::SimpleClientGoalState &state, const motion_control::MotionResultConstPtr &result)
{
  ROS_INFO("action finished");
  switch (result->status) {

  case motion_control::MotionResult::MOTION_STATUS_SUCCESS:
    ROS_INFO("success!");
    break;
  case motion_control::MotionResult::MOTION_STATUS_COLLISION:
    ROS_INFO("collision!");
    break;
  default:
    ROS_INFO("fail code=%d",result->status);
    break;
  }
  state_=STATE_S_WAIT_PATH;

}


void FollowTestNode::feedbackCallback(const motion_control::MotionFeedbackConstPtr& feedback)
{
  ROS_INFO("distance to goal: %f",feedback->dist_goal);
}


void FollowTestNode::activeCallback()
{
  ROS_INFO("motion control active now");
}

bool FollowTestNode::execute()
{
  ros::spin();
  return true;
}

int main(int argc, char** argv )
{
  ros::init(argc,argv, "drivetest");
  ros::NodeHandle nh;
  ros::Time lastTime;

  FollowTestNode node(nh);

  node.execute();
  ROS_INFO( "Quitting... " );
  return 0;
}
