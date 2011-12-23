#include "PathDriver.h"

#include <ramaxxbase/RamaxxMsg.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include "DualAxisDriver.h"
#include <utils/LibUtil/MathHelper.h>

PathDriver::PathDriver(ros::Publisher& cmd_pub, ros::NodeHandle& node)
  : m_has_subgoal(false), m_has_path(false), m_has_odom(false), m_planning_done_(false)
{
  m_path_subscriber = node.subscribe<nav_msgs::Path>
      ("/rs_path", 2, boost::bind(&PathDriver::update_path, this, _1));

  m_command_ramaxx_publisher = cmd_pub;
  m_rs_goal_publisher = node.advertise<geometry_msgs::PoseStamped> ("/rs/goal", 10);

  m_marker_publisher = node.advertise<visualization_msgs::Marker> ("/path_following_marker", 10);

  driver_=new DualAxisDriver(node);

  configure(node);

}

PathDriver::~PathDriver()
{
  delete(driver_);
}

void PathDriver::start (){
  if(m_planning_done_){
    m_poses.clear();
    m_nodes = m_path.poses.size();

    for(int i = 0; i < m_nodes; ++i){
      m_poses.push_back(m_path.poses[i]);
    }
    m_has_path = m_nodes > 0;

    m_has_subgoal = false;
  }
}

void PathDriver::stop (){
  m_has_path = false;
}

/**
  @return state
  */
int PathDriver::execute (MotionFeedback& fb, MotionResult& result){
  if(!m_planning_done_){
    ROS_INFO("path planning still running");
    result.status= MotionResult::MOTION_STATUS_MOVING;
    return MotionResult::MOTION_STATUS_MOVING;

  } else if(!m_has_path){
    ROS_INFO("no path available");
    result.status= MotionResult::MOTION_STATUS_SLAM_FAIL;
    return MotionResult::MOTION_STATUS_SLAM_FAIL;
  }

  double speed=0;
  double front_rad=0.0,rear_rad=0.0;

  if(!m_has_subgoal){
    if(m_poses.size() > 0){
      temp_goal_pose_global_ = m_poses.front();
      temp_goal_pose_global_.header.frame_id = goal_pose_global_.header.frame_id;
      m_poses.pop_front();
      m_has_subgoal = true;

    } else {
      ROS_INFO("path follower: goal reached");
      m_has_path=false;
      result.status=MotionResult::MOTION_STATUS_SUCCESS;
      state_=MotionResult::MOTION_STATUS_SUCCESS;
    }
  }

  // transform goal point into local cs
  geometry_msgs::PoseStamped goal_pose_local;
  try {
    listener_.transformPose("base_link", temp_goal_pose_global_, goal_pose_local);
  }
  catch (tf::TransformException ex){
    ROS_ERROR_STREAM("PathDriver: Cannot transform point -> " << ex.what());
    return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
  }

  // check distance to goal
  Vector3d goal_vec;
  goal_vec.x()=goal_pose_local.pose.position.x;
  goal_vec.y()=goal_pose_local.pose.position.y;

  // goal in reach for 4ws driver?

  if ((goal_vec.head<2>().norm()<m_waypoint_threshold) ) {
    m_has_subgoal = false;
    return execute(fb, result);

  } else {
    // driver drives towards goal
    goal_vec.z()=tf::getYaw(goal_pose_local.pose.orientation);
    driver_->Update(goal_vec);
    driver_->GetCmd(speed, front_rad, rear_rad);
    state_=MotionResult::MOTION_STATUS_MOVING;

    send_arrow_marker(0, "current goal", temp_goal_pose_global_.pose, 0.5, 0.5, 0.5, 1, 0, 0, 1);
  }

  // publish command to robot
  ramaxxbase::RamaxxMsg msg;
  msg.data.resize(3);
  msg.data[0].key=ramaxxbase::RamaxxMsg::CMD_SPEED;
  msg.data[1].key=ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;
  msg.data[2].key=ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG;
  msg.data[0].value=speed;
  msg.data[1].value=angleDeg(front_rad);
  msg.data[2].value=angleDeg(rear_rad);

  m_command_ramaxx_publisher.publish(msg);

  fb.dist_driven = (m_nodes - m_poses.size()) * m_max_waypoint_distance;
  fb.dist_goal = m_poses.size() * m_max_waypoint_distance;

  return state_;
}

void PathDriver::configure (ros::NodeHandle &node){
  node.param<double> ("waypoint_threshold", m_waypoint_threshold, 0.15);
  node.param<double> ("max_waypoint_distance", m_max_waypoint_distance, 0.25);
}

void PathDriver::setGoal (const motion_control::MotionGoal& goal){
  goal_pose_global_.header.frame_id = "/map";
  goal_pose_global_.pose.position.x=goal.x;
  goal_pose_global_.pose.position.y=goal.y;
  goal_pose_global_.pose.position.z=0;
  goal_pose_global_.pose.orientation=
      tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,goal.theta);

  // run reeds shepp
  m_planning_done_ = false;
  m_rs_goal_publisher.publish(goal_pose_global_);

}

void PathDriver::update_path(const nav_msgs::PathConstPtr &path)
{
  m_path = *path;
  m_planning_done_ = true;
  ROS_INFO("pathfollower: path received with %d poses",m_path.poses.size());

  start();
}

void PathDriver::send_arrow_marker(int id, std::string name_space, geometry_msgs::Pose &pose,
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
