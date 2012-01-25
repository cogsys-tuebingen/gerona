#ifndef MOTIONCONTROLNODE_H
#define MOTIONCONTROLNODE_H
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <motion_control/MotionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <Eigen/Core>

class MotionController;
class CalibDriver;
class RsPathDriver;
class FixedDriver;
class SimpleGoalDriver;

class MotionControlNode
{
public:
  MotionControlNode(ros::NodeHandle& node, const std::string& name);
  ~MotionControlNode();

  void odometry_callback (const nav_msgs::OdometryConstPtr &odom);
  void path_callback (const nav_msgs::PathConstPtr &path);
  void action_callback (const geometry_msgs::PoseStampedConstPtr &goal_pose);
  void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
  void goalCallback();
  void preemptCallback();

  void update ();
  void publish();



private:
  void send_arrow_marker(int id, std::string name_space, geometry_msgs::Pose &pose,
                         float scale_x, float scale_y, float scale_z,
                         float r, float g, float b, float a);

  actionlib::SimpleActionServer<motion_control::MotionAction> action_server_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_ramaxx_pub_;

  tf::TransformListener listener_;

  nav_msgs::Odometry odometry_;
  std::string action_name_;

  RsPathDriver *rspath_driver_;
  MotionController *active_ctrl_;
  CalibDriver *calib_driver_;
  SimpleGoalDriver *simple_goal_driver_;
  FixedDriver* fixed_driver_;
};

#endif // MOTIONCONTROLNODE_H
