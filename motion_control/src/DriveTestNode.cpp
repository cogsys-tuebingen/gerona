#include <visualization_msgs/Marker.h>

#include "DriveTestNode.h"
#define RTOD(r)   ((r) * 180 / M_PI)
const double DEFAULT_V = 0.6;

DriveTestNode::DriveTestNode(ros::NodeHandle& node)
  : action_client_("motion_control"),n_(node), state_(START), default_v_(DEFAULT_V)
{
  m_path_subscriber = node.subscribe<nav_msgs::Path>
      ("/rs_path", 2, boost::bind(&DriveTestNode::update_path, this, _1));


  m_goal_subscriber = node.subscribe<geometry_msgs::PoseStamped>
      ("/goal",1, boost::bind(&DriveTestNode::update_goal, this, _1));

  m_marker_publisher = node.advertise<visualization_msgs::Marker> ("/path_following_marker", 10);




}


DriveTestNode::~DriveTestNode()
{
  // nothing to do
}


void DriveTestNode::doneCallback(const actionlib::SimpleClientGoalState &state, const motion_control::MotionResultConstPtr &result)
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

}


void DriveTestNode::feedbackCallback(const motion_control::MotionFeedbackConstPtr& feedback)
{
  ROS_INFO("distance to goal: %f",feedback->dist_goal);
}


void DriveTestNode::activeCallback()
{
  ROS_INFO("motion control active now");
}


void DriveTestNode::update_goal(const geometry_msgs::PoseStampedConstPtr &goal_pose)
{
  ROS_INFO("DriveTest:GOAL received");
  if (!goal_pose->header.frame_id.compare("/base_link")||!goal_pose->header.frame_id.compare("base_link")) {
    // transform to global pose
    ROS_INFO("pathfollower: new local goal %f %f %f",goal_pose->pose.position.x,goal_pose->pose.position.y,
          RTOD(tf::getYaw(goal_pose->pose.orientation)));
    listener_.transformPose("/map",ros::Time(0),*goal_pose,"/base_link",goal_pose_global_);
    ROS_INFO("pathfollower: new global goal %f %f %f",goal_pose_global_.pose.position.x,
           goal_pose_global_.pose.position.y,
           RTOD(tf::getYaw(goal_pose_global_.pose.orientation)));
    motion_control::MotionGoal goal;
    goal.x=goal_pose_global_.pose.position.x;
    goal.y=goal_pose_global_.pose.position.y;
    goal.theta=tf::getYaw(goal_pose_global_.pose.orientation);
    goal.v=default_v_;
    goal.beta=0;
    goal.mode=motion_control::MotionGoal::MOTION_TO_GOAL;
    action_client_.sendGoal(goal,boost::bind(&DriveTestNode::doneCallback,this,_1,_2),boost::bind(&DriveTestNode::activeCallback,this),
                          boost::bind(&DriveTestNode::feedbackCallback,this,_1));
  } else {
    ROS_WARN("pathfollower: unknown frame id %s",goal_pose->header.frame_id.c_str());
  }

}


bool DriveTestNode::execute()
{
  ros::spin();
  return true;
}

void DriveTestNode::update_path(const nav_msgs::PathConstPtr &path)
{
  /*
  m_path = *path;
  m_has_path = true;
  ROS_INFO("pathfollower: path received with %d poses",m_path.poses.size());

  m_poses.clear();
  for(int i = 0; i < (int) path->poses.size(); ++i){
    m_poses.push_back(path->poses[i]);
  }

  */
}

int main(int argc, char** argv )
{
  ros::init(argc,argv, "drivetest");
  ros::NodeHandle nh;
  ros::Time lastTime;

  DriveTestNode node(nh);

  node.execute();
  ROS_INFO( "Quitting... " );
  return 0;
}
