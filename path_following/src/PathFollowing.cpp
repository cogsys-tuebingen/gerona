#include "PathFollowing.h"

#include <ramaxxbase/RamaxxMsg.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include "DualAxisDriver.h"

PathFollowing::PathFollowing(ros::NodeHandle& node)
  : m_has_path(false), m_has_odom(false), has_goal_(false), kp(1.0f), ki(0.0f), kd(0.0f), m_state(INVALID)
{
  m_path_subscriber = node.subscribe<nav_msgs::Path>
      ("/rs_path", 2, boost::bind(&PathFollowing::update_path, this, _1));

  m_odom_subscriber = node.subscribe<nav_msgs::Odometry>
      ("/odom", 100, boost::bind(&PathFollowing::update_odometry, this, _1));

  m_command_ramaxx_publisher = node.advertise<ramaxxbase::RamaxxMsg>
      ("/ramaxx_cmd", 100);

  m_goal_subscriber = node.subscribe<geometry_msgs::PoseStamped>
      ("/goal",1, boost::bind(&PathFollowing::update_goal, this, _1));

  m_marker_publisher = node.advertise<visualization_msgs::Marker> ("/path_following_marker", 10);

  m_last_time = ros::Time::now().toSec();


  node.param<double> ("forward_speed", m_forward_speed, 0.5);
  node.param<double> ("backward_speed", m_backward_speed, -0.4);
  node.param<double> ("waypoint_threshold", m_waypoint_threshold, 0.1);
  node.param<double> ("steer_max", m_steer_max, 20.0f);


  driver_=new DualAxisDriver(node);

}

PathFollowing::~PathFollowing()
{
  delete(driver_);
}


void PathFollowing::update_goal (const geometry_msgs::PoseStampedConstPtr &goal)
{
  if (!goal->header.frame_id.compare("/base_link")||!goal->header.frame_id.compare("base_link")) {
    // transform to global pose
    ROS_INFO("pathfollower: new local goal %f %f %f",goal->pose.position.x,goal->pose.position.y,
                 RTOD(tf::getYaw(goal->pose.orientation)));

    listener_.transformPose("/map",ros::Time(0),*goal,"/base_link",goal_pose_global_);

    ROS_INFO("pathfollower: new global goal %f %f %f",goal_pose_global_.pose.position.x,
             goal_pose_global_.pose.position.y,
             RTOD(tf::getYaw(goal_pose_global_.pose.orientation)));
    has_goal_=true;

  } else {
    ROS_WARN("pathfollower: unknown frame id %s",goal->header.frame_id.c_str());
  }


}


void PathFollowing::update ()
{
  double speed=0;
  double front_rad=0.0,rear_rad=0.0;
  if (has_goal_) {
    // transform goal point into local cs
    geometry_msgs::PoseStamped goal_pose_local;
    listener_.transformPose("/base_link",ros::Time(0),goal_pose_global_,"/map",goal_pose_local);
    // check distance to goal
    Vector3d goal_vec;
    goal_vec.x()=goal_pose_local.pose.position.x;
    goal_vec.y()=goal_pose_local.pose.position.y;

    // goal in reach for 4ws driver?

    if ((goal_vec.head<2>().norm()<0.3) ) {
      // goal reached
      ROS_INFO("path follower: goal reached");
      has_goal_=false;
    } else {
      // driver drives towards goal
      goal_vec.z()=tf::getYaw(goal_pose_local.pose.orientation);
      driver_->Update(goal_vec);
      driver_->GetCmd(speed, front_rad, rear_rad);
    }
  }
  // publish command to robot
  ramaxxbase::RamaxxMsg msg;
  msg.data.resize(3);
  msg.data[0].key=ramaxxbase::RamaxxMsg::CMD_SPEED;
  msg.data[1].key=ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;
  msg.data[2].key=ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG;
  msg.data[0].value=speed;
  msg.data[1].value=RTOD(front_rad);
  msg.data[2].value=RTOD(rear_rad);

  m_command_ramaxx_publisher.publish(msg);

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
  ROS_INFO("pathfollower: path received with %d poses",m_path.poses.size());

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

  ros::Rate rate(50);

  while( ros::ok() )
  {
    ros::spinOnce();
    node.update();
    rate.sleep();
  }

  return 0;
}
