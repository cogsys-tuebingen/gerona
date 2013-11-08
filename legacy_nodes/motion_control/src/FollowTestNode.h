#ifndef FOLLOWTESTNODE_H
#define FOLLOWTESTNODE_H
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <utils/LibBotHardware/DataListener.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_control/MotionAction.h>

#include "Stopwatch.h"

#include <string>
#include <queue>



class FollowTestNode
{
  enum {
    STATE_S_START,
    STATE_S_WAIT_CIRCLE,
    STATE_S_WAIT_PATH,
    STATE_S_DRIVE_PATH
  };
public:
  FollowTestNode(ros::NodeHandle& n );
  ~FollowTestNode ();
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const motion_control::MotionResultConstPtr& result);
  void feedbackCallback(const motion_control::MotionFeedbackConstPtr& feedback);
  void activeCallback();
  void pathReceived (const nav_msgs::PathConstPtr &path);
  bool execute ();


private:
  actionlib::SimpleActionClient<motion_control::MotionAction> action_client_;
  ros::NodeHandle& n_;
  int state_;
  double default_v_;
  ros::Subscriber path_subscriber_;
  //ros::Publisher ring_goal_publisher_,goal_publisher_;
  tf::TransformListener listener_;

  geometry_msgs::PoseStamped goal_pose_global_;
  bool has_goal_;
  bool has_path_;


};

#endif // FollowTestNode_H
