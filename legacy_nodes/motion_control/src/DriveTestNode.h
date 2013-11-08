#ifndef DRIVETESTNODE_H
#define DRIVETESTNODE_H
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
class ConfigFileReader;
class RobotMission;


class DriveTestNode
{
public:
  DriveTestNode(ros::NodeHandle& n );
  ~DriveTestNode ();
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const motion_control::MotionResultConstPtr& result);
  void feedbackCallback(const motion_control::MotionFeedbackConstPtr& feedback);
  void activeCallback();
  void update_goal (const geometry_msgs::PoseStampedConstPtr &goal_pose);
  void update_path (const nav_msgs::PathConstPtr &path);
  bool execute ();

  enum {
    START,
    MOVING,
    AT_GOAL,
    FAILED
  };

private:
  actionlib::SimpleActionClient<motion_control::MotionAction> action_client_;
  ros::NodeHandle& n_;
  int state_;
  double default_v_;
  ros::Subscriber m_goal_subscriber;
  ros::Subscriber m_path_subscriber;
  ros::Publisher m_marker_publisher;
  tf::TransformListener listener_;

  geometry_msgs::PoseStamped goal_pose_global_;
  bool has_goal_;


};

#endif // DRIVETESTNODE_H
