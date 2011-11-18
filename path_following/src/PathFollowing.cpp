#include "PathFollowing.h"

#include <ramaxxbase/RamaxxMsg.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

PathFollowing::PathFollowing(ros::NodeHandle& n)
  : m_has_path(false), m_has_odom(false), kp(1.0f), ki(0.0f), kd(0.0f), m_state(INVALID)
{
  m_path_subscriber = n.subscribe<nav_msgs::Path>
      ("/rs_path", 2, boost::bind(&PathFollowing::update_path, this, _1));

  m_odom_subscriber = n.subscribe<nav_msgs::Odometry>
      ("/odom", 100, boost::bind(&PathFollowing::update_odometry, this, _1));

  m_command_ramaxx_publisher = n.advertise<ramaxxbase::RamaxxMsg>
      ("/ramaxx_cmd", 100);

  m_marker_publisher = n.advertise<visualization_msgs::Marker> ("/path_following_marker", 10);

  m_last_time = ros::Time::now().toSec();

  n.param<double> ("kp", kp, 1.0);
  n.param<double> ("ki", ki, 0.0);
  n.param<double> ("kd", kd, 0.0);

  n.param<double> ("forward_speed", m_forward_speed, 0.1);
  n.param<double> ("backward_speed", m_backward_speed, 0.1);
  n.param<double> ("waypoint_threshold", m_waypoint_threshold, 0.1);
  n.param<double> ("steer_max", m_steer_max, 20.0f);

  ROS_WARN_STREAM("kp: " << kp << ", ki: " << ki << ", kd: " << kd);

  m_pid = new attempto::PID<double> (0.0, kp, ki, kd);
}

PathFollowing::~PathFollowing()
{
  delete m_pid;
}

void PathFollowing::update_odometry(const nav_msgs::OdometryConstPtr &odom)
{

  tf::StampedTransform transform_map;
  try {
    listener_.lookupTransform("/map", "/odom", ros::Time(0), transform_map);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("Cannot transform point");
    return;
  }

  tf::Vector3 pt;
  pt.setX(odom->pose.pose.position.x);
  pt.setY(odom->pose.pose.position.y);
  tf::Vector3 map_pt = transform_map(pt);
  m_odometry.pose.pose.position.x = map_pt.x();
  m_odometry.pose.pose.position.y = map_pt.y();
  m_odometry.pose.pose.orientation = odom->pose.pose.orientation;
  m_has_odom = true;
}

void PathFollowing::update_path(const nav_msgs::PathConstPtr &path)
{
  m_path = *path;
  m_has_path = true;

  m_poses.clear();
  for(int i = 0; i < (int) path->poses.size(); ++i){
    m_poses.push_back(path->poses[i]);
  }

  m_state = INVALID;
}

void PathFollowing::publish()
{
  if(!m_has_path || !m_has_odom || m_state == AT_GOAL)
    return;

  double dx, dy;

  geometry_msgs::PoseStamped target;
  geometry_msgs::Pose direction, steer_direction;

  float direction_distance;
  double target_angle;
  double orientation = tf::getYaw(m_odometry.pose.pose.orientation);
  double angle_error;

  bool has_next = m_poses.size() > 0;

  // find the next waypoint (discard everyone that is too close)
  while(has_next) {
    target = m_poses.front();

    dx = target.pose.position.x - m_odometry.pose.pose.position.x;
    dy = target.pose.position.y - m_odometry.pose.pose.position.y;

    direction_distance = sqrt(dx*dx + dy*dy);
    target_angle = atan2(dy, dx);
    angle_error = target_angle - orientation;

    if(m_state == INVALID){
      // this is done once, for the first waypoint
      // find out the starting direction
      m_forward = fabs(NORMALIZE(angle_error)) < DTOR(90);
      m_state = MOVING;

    } else {
      if(!m_forward){
        // when moving backwards, the error angle is different...
        angle_error = NORMALIZE(DTOR(180)-angle_error);
      }

      // if the error angle is to big, switch driving direction
      if(fabs(angle_error) > DTOR(90)){
        m_forward = !m_forward;
        if(m_forward)
          ROS_WARN("forward");
        else
          ROS_WARN("backward");
      }

      if(direction_distance < m_waypoint_threshold){
        // if the distance is small enough, choose next waypoint
        m_poses.pop_front();
        has_next = m_poses.size() > 0;

      } else {
        // otherwise keep the current one
        break;
      }
    }
  };

  // no more wayopints?
  if(!has_next) {
    ROS_INFO("goal reached");
    m_state = AT_GOAL;
  } else
    m_state = MOVING;

  if(m_state == MOVING){
    // invariant: the current waypoint is valid

    // time difference for PID controller
    double now = ros::Time::now().toSec();
    double d_time = now - m_last_time;

    // determine driving speed
    double speed = 0.0;
    if(m_forward){
      speed = m_forward_speed;
    } else {
      speed = -m_backward_speed;
    }

    // determine steering angle
    double steer_max = DTOR(m_steer_max);
    double steer = std::max(-steer_max, std::min( steer_max, m_pid->regulate(-angle_error, d_time) ));

    // some rviz visualization
    direction.position = m_odometry.pose.pose.position;
    direction.orientation = tf::createQuaternionMsgFromYaw(target_angle);

    steer_direction.position = direction.position;
    if(m_forward){
      steer_direction.orientation =  tf::createQuaternionMsgFromYaw(orientation+steer);
    } else {
      steer_direction.orientation =  tf::createQuaternionMsgFromYaw(orientation+DTOR(180)-steer);
    }

    send_arrow_marker(0, "target pose", target.pose,
                      0.25f, 1.f, 1.f,
                      0.0f, 0.0f, 1.0f, 1.0f);
    send_arrow_marker(0, "direction to next pose", direction,
                      direction_distance, 1.f, 1.f,
                      1.0f, 0.0f, 0.0f, 1.0f);
    send_arrow_marker(0, "steer direction from pid", steer_direction,
                      fabs(speed), 1.f, 1.f,
                      0.0f, 1.0f, 0.0f, 1.0f);

    ramaxxbase::RamaxxMsg msg;
    ramaxxbase::key_val_pair msg_speed, msg_steer;

    msg_speed.key = ramaxxbase::RamaxxMsg::CMD_SPEED;
    msg_steer.key = ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;

    msg_speed.value = speed;
    msg_steer.value = RTOD(steer);

    msg.data.push_back(msg_speed);
    msg.data.push_back(msg_steer);

    m_command_ramaxx_publisher.publish(msg);
  }
}

void PathFollowing::send_arrow_marker(int id, std::string name_space, geometry_msgs::Pose &pose,
                                      float scale_x, float scale_y, float scale_z,
                                      float r, float g, float b, float a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";

  marker.ns = name_space;
  marker.id = id;

  marker.type = visualization_msgs::Marker::ARROW;

  marker.action = visualization_msgs::Marker::ADD;

  marker.pose = pose;

  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.lifetime = ros::Duration(3);

  m_marker_publisher.publish(marker);
}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "path_follower");
  ros::NodeHandle n("~");

  PathFollowing node(n);

  ros::Rate rate(200);

  while( ros::ok() )
  {
    ros::spinOnce();

    node.publish();
    rate.sleep();
  }

  return 0;
}
