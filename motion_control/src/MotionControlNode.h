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
#include <utils/LibUtil/LowPassFilter.h>

class MotionController;
class CalibDriver;
class PatternDriver;
class FixedDriver;
class SimpleGoalDriver;
using namespace Eigen;
class MotionControlNode
{
public:
  MotionControlNode(ros::NodeHandle& node, const std::string& name);
  ~MotionControlNode();

  void odometryCallback (const nav_msgs::OdometryConstPtr &odom);
  void path_callback (const nav_msgs::PathConstPtr &path);
  void action_callback (const geometry_msgs::PoseStampedConstPtr &goal_pose);
  void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
  void sonarCallback(const sensor_msgs::PointCloudConstPtr &data);
  void goalCallback();
  void preemptCallback();

  void update ();
  void publish();
  bool getWorldPose(Vector3d& pose, geometry_msgs::Pose* pose_out = NULL) const;

  bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
  bool transformToLocal(const geometry_msgs::PoseStamped& global, Vector3d& local );
  ros::NodeHandle& getNodeHandle() {return nh_;}
private:
  void send_arrow_marker(int id, std::string name_space, geometry_msgs::Pose &pose,
                         float scale_x, float scale_y, float scale_z,
                         float r, float g, float b, float a);

  ros::NodeHandle& nh_;
  ros::NodeHandle nh_private_;

  actionlib::SimpleActionServer<motion_control::MotionAction> action_server_;
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber sonar_sub_;
  ros::Publisher cmd_ramaxx_pub_;

  tf::TransformListener pose_listener_;

  nav_msgs::Odometry odometry_;
  LowPassFilter<float> speed_filter_;
  std::string action_name_;
  std::string world_frame_;
  std::string robot_frame_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_topic_;

  MotionController *active_ctrl_;
  CalibDriver *calib_driver_;
  SimpleGoalDriver *simple_goal_driver_;
  PatternDriver* pattern_driver_;
  FixedDriver* fixed_driver_;
  MotionController* path_driver_;
};

#endif // MOTIONCONTROLNODE_H
