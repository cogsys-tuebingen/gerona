#include <visualization_msgs/Marker.h>

#include "FollowTestNode.h"
#define RTOD(r)   ((r) * 180 / M_PI)
const double DEFAULT_V = 0.6;

FollowTestNode::FollowTestNode(ros::NodeHandle& nh)
  : action_client_("/motion_control", true),n_(nh), state_(STATE_S_START),
    has_path_(false)
{
  std::string topic = "/path_planner/path";
  nh.param("topic", topic, topic);
  path_subscriber_ = nh.subscribe<nav_msgs::Path>
      (topic, 1, boost::bind(&FollowTestNode::pathReceived, this, _1));

  ros::param::param<double>("~velocity", default_v_, DEFAULT_V);
  ROS_INFO("Set velocity to %g", default_v_);

  ROS_INFO("sampling initiated");
  state_ = STATE_S_WAIT_PATH;
}


FollowTestNode::~FollowTestNode()
{
  // nothing to do
}



void FollowTestNode::pathReceived(const nav_msgs::PathConstPtr &path)
{
  action_client_.waitForServer();
  if (state_==STATE_S_DRIVE_PATH) {
      ROS_WARN("aborting path following!");
      action_client_.cancelAllGoals();
  }

    ROS_INFO("pathfollower: path received with %zu poses",path->poses.size());
    if (path->poses.size()>0) {
      motion_control::MotionGoal goal;
      goal.mode=motion_control::MotionGoal::MOTION_FOLLOW_PATH;
      goal.pos_tolerance=0.1;
      goal.v=default_v_;
      goal.target_v=0;
      goal.path=*path;
      goal.path_topic="";

      if(!action_client_.isServerConnected()) {
          ROS_ERROR("cannot send the motion control request, server offline.");
          return;
      }
      action_client_.sendGoal(goal,boost::bind(&FollowTestNode::doneCallback,this,_1,_2),
                              boost::bind(&FollowTestNode::activeCallback,this),
                              boost::bind(&FollowTestNode::feedbackCallback,this,_1));
      state_=STATE_S_DRIVE_PATH;
    } else {
      ROS_ERROR("no good path received wait for new circle");
      state_=STATE_S_WAIT_PATH;
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
//  ROS_INFO("distance to goal: %f",feedback->dist_goal);
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
