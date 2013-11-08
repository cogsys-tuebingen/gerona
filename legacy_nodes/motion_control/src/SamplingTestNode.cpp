#include <visualization_msgs/Marker.h>

#include "SamplingTestNode.h"
#define RTOD(r)   ((r) * 180 / M_PI)
const double DEFAULT_V = 0.6;

SamplingTestNode::SamplingTestNode(ros::NodeHandle& nh)
  : action_client_("motion_control"),n_(nh), state_(STATE_S_START), default_v_(DEFAULT_V),
    has_goal_(false),has_circle_(false),has_path_(false)
{

  circle_subscriber_ =
      nh.subscribe<geometry_msgs::Point> ("/largest_circle", 1,
                        boost::bind(&SamplingTestNode::circleReceived, this, _1));


  path_subscriber_ = nh.subscribe<nav_msgs::Path>
      ("/rs_path", 1, boost::bind(&SamplingTestNode::pathReceived, this, _1));


  ring_goal_publisher_ = nh.advertise<geometry_msgs::Point>
      ("/rs/ring_goal",10);
  goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>
        ("/rs/goal",10);
  ROS_INFO("sampling inited");
  state_=STATE_S_WAIT_CIRCLE;

}


SamplingTestNode::~SamplingTestNode()
{
  // nothing to do
}


void SamplingTestNode::circleReceived(const geometry_msgs::PointConstPtr& circle)
{
  if (state_==STATE_S_WAIT_CIRCLE) {
    double radius = circle->z;
    if (radius>1.0) {
      ROS_INFO_STREAM("circle received r="<<radius<< " cx="<<circle->x<< " cy="<<circle->y);
      geometry_msgs::Point p=*circle;
      p.z=p.z-0.3;
      ring_goal_publisher_.publish(p);
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x=circle->x;
      pose.pose.position.y=circle->y;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0),
                            pose.pose.orientation);
//      goal_publisher_.publish(pose);
      state_=STATE_S_WAIT_PATH;
    }
  }
}

void SamplingTestNode::pathReceived(const nav_msgs::PathConstPtr &path)
{
  if (state_==STATE_S_WAIT_PATH) {
    ROS_INFO("pathfollower: path received with %d poses",path->poses.size());

    for(unsigned i = 0; i < path->poses.size(); ++i){
      ROS_INFO("x: %f y: %f ",path->poses[i].pose.position.x,path->poses[i].pose.position.y);
    }
    exit(0);
  }
  /*
  m_path = *path;
  m_has_path = true;

  m_poses.clear();
  for(int i = 0; i < (int) path->poses.size(); ++i){
    m_poses.push_back(path->poses[i]);
  }

  */
}

void SamplingTestNode::doneCallback(const actionlib::SimpleClientGoalState &state, const motion_control::MotionResultConstPtr &result)
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


void SamplingTestNode::feedbackCallback(const motion_control::MotionFeedbackConstPtr& feedback)
{
  ROS_INFO("distance to goal: %f",feedback->dist_goal);
}


void SamplingTestNode::activeCallback()
{
  ROS_INFO("motion control active now");
}

/*
void SamplingTestNode::updateGoal(const geometry_msgs::PoseStampedConstPtr &goal_pose)
{
  ROS_INFO("DriveTest:GOAL received");

  motion_control::MotionGoal goal;
  goal.v=default_v_;
  goal.beta=0;

  if (!goal_pose->header.frame_id.compare("/map")||!goal_pose->header.frame_id.compare("map")) {
    // transform to global pose
    goal_pose_global_ = *goal_pose;
    ROS_INFO("pathfollower: new map goal %f %f %f",goal_pose_global_.pose.position.x,
           goal_pose_global_.pose.position.y,
           RTOD(tf::getYaw(goal_pose_global_.pose.orientation)));

    goal.mode=motion_control::MotionGoal::MOTION_FOLLOW_PATH;

  } else if (!goal_pose->header.frame_id.compare("/base_link")||!goal_pose->header.frame_id.compare("base_link")) {
    // transform to global pose
    ROS_INFO("pathfollower: new local goal %f %f %f",goal_pose->pose.position.x,goal_pose->pose.position.y,
          RTOD(tf::getYaw(goal_pose->pose.orientation)));
    listener_.transformPose("/map",ros::Time(0),*goal_pose,"/base_link",goal_pose_global_);
    ROS_INFO("pathfollower: new global goal %f %f %f",goal_pose_global_.pose.position.x,
           goal_pose_global_.pose.position.y,
           RTOD(tf::getYaw(goal_pose_global_.pose.orientation)));
    goal.mode=motion_control::MotionGoal::MOTION_TO_GOAL;

  } else {
    ROS_WARN("pathfollower: unknown frame id %s",goal_pose->header.frame_id.c_str());
    return;
  }

  goal.x=goal_pose_global_.pose.position.x;
  goal.y=goal_pose_global_.pose.position.y;
  goal.theta=tf::getYaw(goal_pose_global_.pose.orientation);

  action_client_.sendGoal(goal,boost::bind(&SamplingTestNode::doneCallback,this,_1,_2),boost::bind(&SamplingTestNode::activeCallback,this),
                        boost::bind(&SamplingTestNode::feedbackCallback,this,_1));

}
*/

bool SamplingTestNode::execute()
{
  ROS_INFO("sampling executing");
  ros::spin();
  return true;
}

int main(int argc, char** argv )
{
  ros::init(argc,argv, "drivetest");
  ros::NodeHandle nh;
  ros::Time lastTime;

  SamplingTestNode node(nh);

  node.execute();
  ROS_INFO( "Quitting... " );
  return 0;
}
