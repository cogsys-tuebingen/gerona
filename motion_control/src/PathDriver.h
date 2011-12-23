#ifndef PATHDRIVER_H
#define PATHDRIVER_H

#include <Pid.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <Eigen/Core>

#include "MotionController.h"

// forward declarations

class CarLikeDriver;
using namespace Eigen;

class PathDriver : public MotionController
{
public:
  PathDriver(ros::Publisher& cmd_pub, ros::NodeHandle& n);
  virtual ~PathDriver();

  virtual void start ();
  virtual void stop ();
  virtual int getType () {
    return motion_control::MotionGoal::MOTION_FOLLOW_PATH;
  }

  /**
    @return state
    */
  virtual int execute (MotionFeedback& fb, MotionResult& result);
  virtual void configure (ros::NodeHandle &node);
  virtual void setGoal (const motion_control::MotionGoal& goal);

  void update_path (const nav_msgs::PathConstPtr &path);

private:
  void send_arrow_marker(int id, std::string name_space, geometry_msgs::Pose &pose,
                         float scale_x, float scale_y, float scale_z,
                         float r, float g, float b, float a);

  ros::Subscriber m_path_subscriber;

  ros::Publisher m_rs_goal_publisher;
  ros::Publisher m_command_ramaxx_publisher;
  ros::Publisher m_marker_publisher;

  tf::TransformListener listener_;

  nav_msgs::Path m_path;
  int m_nodes;
  std::deque<geometry_msgs::PoseStamped> m_poses;


  geometry_msgs::PoseStamped goal_pose_global_;
  geometry_msgs::PoseStamped temp_goal_pose_global_;

  CarLikeDriver *driver_;

  bool m_has_subgoal;
  bool m_has_path;
  bool m_has_odom;
  bool m_planning_done_;
  bool m_forward;

  double m_waypoint_threshold;
  double m_max_waypoint_distance;

  int state_;
};

#endif // PATHDRIVER_H
