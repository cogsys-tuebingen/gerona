#ifndef PATHFOLLOWING_H
#define PATHFOLLOWING_H

#include <Pid.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <Eigen/Core>

// forward declarations

class CarLikeDriver;
using namespace Eigen;

#define RTOD(r)   ((r) * 180 / M_PI)
#define DTOR(d)   ((d) * M_PI / 180)
#define NORMALIZE(z)   atan2(sin(z), cos(z))



class PathFollowing
{
public:
  PathFollowing(ros::NodeHandle& n);
  virtual ~PathFollowing();

  void update_odometry (const nav_msgs::OdometryConstPtr &odom);
  void update_path (const nav_msgs::PathConstPtr &path);
  void update_goal (const geometry_msgs::PoseStampedConstPtr &goal_pose);

  void update ();
  void publish();


  enum STATE {
    INVALID,
    MOVING,
    AT_GOAL
  };

private:
  void send_arrow_marker(int id, std::string name_space, geometry_msgs::Pose &pose,
                         float scale_x, float scale_y, float scale_z,
                         float r, float g, float b, float a);

  ros::Subscriber m_path_subscriber;
  ros::Subscriber m_odom_subscriber;
  ros::Subscriber m_goal_subscriber;

  ros::Publisher m_command_ramaxx_publisher;
  ros::Publisher m_marker_publisher;

  tf::TransformListener listener_;

  nav_msgs::Odometry m_odometry;
  nav_msgs::Path m_path;
  std::deque<geometry_msgs::PoseStamped> m_poses;


  geometry_msgs::PoseStamped goal_pose_global_;

  CarLikeDriver *driver_;


  bool m_has_path;
  bool m_has_odom;
  bool has_goal_;
  bool m_forward;

  double m_last_time;

  double kp;
  double ki;
  double kd;
  attempto::PID<double> * m_pid;

  double m_forward_speed;
  double m_backward_speed;
  double m_waypoint_threshold;
  double m_steer_max;

  STATE m_state;
};

#endif // PATHFOLLOWING_H
