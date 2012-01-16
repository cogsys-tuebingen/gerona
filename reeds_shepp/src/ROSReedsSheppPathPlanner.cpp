/*
 * ROSReedsSheppPathPlanner.cpp
 *
 *  Created on: Aug 19, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */


#include <utils/LibPath/sampling/SamplingPlanner.h>
#include <utils/LibPath/sampling/RingGoalRegion.h>
#include <utils/LibPath/sampling/CentroidRadiusGoalRegion.h>

#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <deque>
#include "ROSReedsSheppPathPlanner.h"

using namespace lib_path;

ROSReedsSheppPathPlanner::ROSReedsSheppPathPlanner(const ros::NodeHandle &n, const bool silent_mode)
  : m_node_handle(n), m_silent_mode(silent_mode), m_has_curve(false), m_has_goal(false), m_has_odom(false), m_has_map(false)
{
  // Read parameters from rosparam server
  init_parameters();

  // Parses all possible RS-patterns
  generate_rs_patterns();

  // Sets cost/threshold/... parameters of RS generator
  set_generator_parameters();

  if (!m_silent_mode){
    // Initialize all necessary subscriber and publisher objects
    init_subscribers();
    init_publishers();
  }
}


void ROSReedsSheppPathPlanner::generate_rs_patterns()
{
  // generate Reeds-Shepp patterns
  // possible characters:
  //  L - left curve
  //  R - right curve
  //  S - straight
  //  | - change driving direction (forward <-> backward)

  // TODO: bug in lines
  m_rs_generator.parse("LSL");
  m_rs_generator.parse("LSR");
  m_rs_generator.parse("RSL");
  m_rs_generator.parse("RSR");

  m_rs_generator.parse("|LSL");
  m_rs_generator.parse("|LSR");
  m_rs_generator.parse("|RSL");
  m_rs_generator.parse("|RSR");

  m_rs_generator.parse("|L|SL");
  m_rs_generator.parse("|L|SR");
  m_rs_generator.parse("|R|SL");
  m_rs_generator.parse("|R|SR");

  m_rs_generator.parse("LS|L");
  m_rs_generator.parse("LS|R");
  m_rs_generator.parse("RS|L");
  m_rs_generator.parse("RS|R");

  m_rs_generator.parse("L|SL");
  m_rs_generator.parse("L|SR");
  m_rs_generator.parse("R|SL");
  m_rs_generator.parse("R|SR");

  m_rs_generator.parse("L|S|L");
  m_rs_generator.parse("L|S|R");
  m_rs_generator.parse("R|S|L");
  m_rs_generator.parse("R|S|R");

  m_rs_generator.parse("|LS|L");
  m_rs_generator.parse("|LS|R");
  m_rs_generator.parse("|RS|L");
  m_rs_generator.parse("|RS|R");

  m_rs_generator.parse("|L|S|L");
  m_rs_generator.parse("|L|S|R");
  m_rs_generator.parse("|R|S|L");
  m_rs_generator.parse("|R|S|R");

  ///// CCC

  //    L(a) L(b)      = L(a+b)
  //    L(a) L(b) L(c) = L(a+b+c)

  m_rs_generator.parse("LRL");
  m_rs_generator.parse("RLR");

  // C|C|C
  m_rs_generator.parse("L|R|L");
  m_rs_generator.parse("R|L|R");
  m_rs_generator.parse("|L|R|L");
  m_rs_generator.parse("|R|L|R");

  // CC|C
  m_rs_generator.parse("LR|L");
  m_rs_generator.parse("RL|R");
  m_rs_generator.parse("|LR|L");
  m_rs_generator.parse("|RL|R");

  // C|CC
  m_rs_generator.parse("L|RL");
  m_rs_generator.parse("R|LR");
  m_rs_generator.parse("|L|RL");
  m_rs_generator.parse("|R|LR");

  m_rs_generator.parse("LR(b)|L(b)R");
  m_rs_generator.parse("|LR(b)|L(b)R");
  m_rs_generator.parse("RL(b)|R(b)L");
  m_rs_generator.parse("|RL(b)|R(b)L");

  m_rs_generator.parse("L|R(b)L(b)|R");
  m_rs_generator.parse("R|L(b)R(b)|L");
  m_rs_generator.parse("|L|R(b)L(b)|R");
  m_rs_generator.parse("|R|L(b)R(b)|L");
}


void ROSReedsSheppPathPlanner::init_parameters()
{
  m_node_handle.param<double> ("circle_radius", m_circle_radius, 1.0f);
  m_node_handle.param<double> ("max_waypoint_distance", m_max_waypoint_distance, 0.25f);
  m_node_handle.param<double> ("cost_backwards", m_cost_backwards, 1.0f);
  m_node_handle.param<double> ("cost_forwards", m_cost_forwards, 1.0f);
  m_node_handle.param<double> ("cost_curve", m_cost_curve, 1.0f);
  m_node_handle.param<double> ("cost_straight", m_cost_straight, 1.0f);
  m_node_handle.param<int> ("threshold_min", m_threshold_min, 0);
  m_node_handle.param<int> ("threshold_max", m_threshold_max, 100);
  m_node_handle.param<std::string> ("map_topic", m_map_topic, "/map");
  m_node_handle.param<std::string> ("publish_frame", m_publish_frame, "/map");
  m_node_handle.param<std::string> ("goal_topic", m_goal_topic, "/rs/goal");
  m_node_handle.param<std::string> ("ring_goal_topic", m_ring_goal_topic, "/rs/ring_goal");
  m_node_handle.param<std::string> ("centroid_goal_topic", m_centroid_goal_topic, "/rs/centroid_goal");
}


void ROSReedsSheppPathPlanner::set_generator_parameters()
{
  m_rs_generator.set_circle_radius(m_circle_radius);
  m_rs_generator.set_max_waypoint_distance(m_max_waypoint_distance);

  m_rs_generator.set_cost_forwards(m_cost_forwards);
  m_rs_generator.set_cost_backwards(m_cost_backwards);
  m_rs_generator.set_cost_curve(m_cost_curve);
  m_rs_generator.set_cost_straight(m_cost_straight);
}


void ROSReedsSheppPathPlanner::init_subscribers()
{
  m_goal_pos_subscriber = m_node_handle.subscribe<geometry_msgs::PoseStamped>
      (m_goal_topic, 1, boost::bind(&ROSReedsSheppPathPlanner::update_goal, this, _1));

  m_ring_goal_subscriber = m_node_handle.subscribe<geometry_msgs::Point>
      (m_ring_goal_topic, 1, boost::bind(&ROSReedsSheppPathPlanner::update_ring_goal, this, _1));

  m_centroid_goal_subscriber = m_node_handle.subscribe<geometry_msgs::Point>
      (m_centroid_goal_topic, 1, boost::bind(&ROSReedsSheppPathPlanner::update_centroid_goal, this, _1));

  m_odom_subscriber = m_node_handle.subscribe<nav_msgs::Odometry>
      ("/odom", 100, boost::bind(&ROSReedsSheppPathPlanner::update_odometry, this, _1));

  m_map_subscriber = m_node_handle.subscribe<nav_msgs::OccupancyGrid>
      (m_map_topic, 2, boost::bind(&ROSReedsSheppPathPlanner::update_map, this, _1));
}


void ROSReedsSheppPathPlanner::init_publishers()
{
  m_path_publisher = m_node_handle.advertise<nav_msgs::Path> ("/rs_path", 10);

  m_marker_publisher = m_node_handle.advertise<visualization_msgs::Marker> ("/rs_marker", 200);
}


void ROSReedsSheppPathPlanner::update_goal(const geometry_msgs::PoseStampedConstPtr &goal)
{
  m_goal_world.x = goal->pose.position.x;
  m_goal_world.y = goal->pose.position.y;
  m_goal_world.theta = tf::getYaw(goal->pose.orientation);
  m_has_goal = true;

  calculate();
}







void ROSReedsSheppPathPlanner::update_odometry(const nav_msgs::OdometryConstPtr &odom)
{
  tf::StampedTransform transform_map;
  try {
    listener_.lookupTransform("/map", "/odom", ros::Time(0), transform_map);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("[ReedsShepp] Cannot transform point");
    return;
  }

  tf::Vector3 pt (odom->pose.pose.position.x,
                  odom->pose.pose.position.y,
                  0.f);
  tf::Vector3 map_pt = transform_map(pt);
  m_odom_world.x = map_pt.x();
  m_odom_world.y = map_pt.y();
  m_odom_world.theta = tf::getYaw(odom->pose.pose.orientation);

  m_has_odom = true;
}


void ROSReedsSheppPathPlanner::update_map(const nav_msgs::OccupancyGridConstPtr &map)
{
  m_map.width = map->info.width;
  m_map.height = map->info.height;
  m_map.origin.x = map->info.origin.position.x;
  m_map.origin.y = map->info.origin.position.y;
  m_map.resolution = map->info.resolution;
  m_map.data = map->data;
  m_map.threshold_min = m_threshold_min;
  m_map.threshold_max = m_threshold_max;
  m_has_map = true;
}


void ROSReedsSheppPathPlanner::update_ring_goal(const geometry_msgs::PointConstPtr &ring_goal)
{
  ROS_INFO("update ring goal");
  Point2d center;
  center.x=ring_goal->x;
  center.y=ring_goal->y;
  double radius=ring_goal->z;
  double width=0.05;
  RingGoalRegion goal(center,radius,width);
  SamplingPlanner planner(&m_rs_generator,&m_map);
  ROS_INFO("calc sampling path from %f:%f to ring with center %f:%f and radius %f",
           m_odom_world.x,m_odom_world.y,center.x,center.y,radius);
  Pose2d odom_map = pos2map(m_odom_world, m_map);

  int8_t startpos_val=m_map.getValue(odom_map.x,odom_map.y);
  for (int dx=-3;dx<3;++dx) {
    for (int dy=-3;dy<3;++dy) {
      m_map.setValue(odom_map.x+dx,odom_map.y+dx,m_threshold_min);
    }
  }
  Curve* curve=planner.createPath(m_odom_world,&goal,20);
  m_map.setValue(odom_map.x,odom_map.y,startpos_val);

  if (curve && curve->is_valid()) {
    ROS_INFO("Publish curve with cost %f", curve->weight());
    publishCurve(curve);
  } else {
    ROS_INFO("No path found.");
    send_empty_path();
  }

  if (curve)
    delete curve;
}


void ROSReedsSheppPathPlanner::update_centroid_goal(const geometry_msgs::PointConstPtr &centroid_goal)
{
  ROS_INFO("update centroid goal");

  Point2d src;
  src.x = m_odom_world.x;
  src.y = m_odom_world.y;

  Point2d center;
  center.x = centroid_goal->x;
  center.y = centroid_goal->y;

  double radius = centroid_goal->z;
  double angle_rad = M_PI_2;

  CentroidRadiusGoalRegion goal (src, center, radius, angle_rad);
  SamplingPlanner planner(&m_rs_generator,&m_map);

  cout << "Odom " << m_odom_world.x << " " << m_odom_world.y << " " << m_odom_world.theta << endl;
  cout << "Goal " << centroid_goal->x << " " << centroid_goal->y << " " << centroid_goal->z << endl;
  Pose2d tha_poser;
  while (goal.getNextGoal (tha_poser)) {
    cout << "Pose: " << tha_poser.x << " " << tha_poser.y  << " " << tha_poser.theta * 180.0 / M_PI << endl;
  }

  ROS_INFO("calc sampling path from %f:%f to centroid at %f:%f in distance %f",
           m_odom_world.x, m_odom_world.y, center.x, center.y, radius);

  Pose2d odom_map = pos2map(m_odom_world, m_map);
  int8_t startpos_val = m_map.getValue (odom_map.x, odom_map.y);
  for (int dx = -3; dx < 3; ++dx) {
    for (int dy = -3; dy < 3; ++dy) {
      m_map.setValue (odom_map.x + dx, odom_map.y + dx, m_threshold_min);
    }
  }

  Curve *curve = planner.createPath (m_odom_world, &goal, 10);
  m_map.setValue(odom_map.x,odom_map.y,startpos_val);

  if (curve && curve->is_valid()) {
    ROS_INFO ("Publish curve with cost %f", curve->weight());
    publishCurve (curve);
  } else {
    ROS_INFO ("No path found.");
    send_empty_path ();
  }

  if (curve)
    delete curve;
}


void ROSReedsSheppPathPlanner::publishCurve (Curve * curve)
{
  if (!m_silent_mode){
    ROS_INFO("Found a path. Cost:%f ", curve->weight());

    // start pos
    send_arrow_marker(0, m_odom_world,
                      0.1, 0.3, 0.3,
                      1.0, 0.0, 0.0, 1.0f);

    // goal pos
    send_arrow_marker(1, m_goal_world,
                      0.1, 0.3, 0.3,
                      0.0, 1.0, 0.0, 1.0f);
  }

  // paths
  nav_msgs::Path path;

  path.header.frame_id = m_publish_frame;
  int id = 2;

  curve->reset_iteration();

  while(curve->has_next()){
    Pose2d next_map = curve->next();
    Pose2d next_world = map2pos(next_map, m_map);

    // Next pose along path
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = next_world.x;
    pose.pose.position.y = next_world.y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_world.theta);

    path.poses.push_back(pose);

    if (!m_silent_mode){
      // marker
      send_arrow_marker(id, next_world,
                        0.05, 0.3, 0.3,
                        0.0, 0.0, 1.0, 0.7f);
      id++;
    }
  }

  m_last_weight = curve->weight();
  m_last_path = path;
  m_has_curve = true;

  if (!m_silent_mode)
    m_path_publisher.publish(m_last_path);
}


void ROSReedsSheppPathPlanner::calculate()
{
  ROS_INFO("calc: %d %d %d",m_has_goal,m_has_odom,m_has_map);
  if(m_has_goal && m_has_odom && m_has_map ){
    start_timer();

    Pose2d odom_map = pos2map(m_odom_world, m_map);
    Pose2d goal_map = pos2map(m_goal_world, m_map);

    if (!m_silent_mode){
      ROS_INFO ("map: resolution: %2.2f, origin: (%2.2f, %2.2f)", m_map.resolution, m_map.origin.x, m_map.origin.y);
      ROS_INFO ("     width: %d, height: %d", m_map.width, m_map.height);
      ROS_INFO ("searching path:");
      ROS_INFO ("world: from: (%2.2f, %2.2f) to: (%2.2f, %2.2f)", m_odom_world.x, m_odom_world.y, m_goal_world.x, m_goal_world.y);
      ROS_INFO ("map:   from: (%2.2f, %2.2f) to: (%2.2f, %2.2f)", odom_map.x, odom_map.y , goal_map.x, goal_map.y);
    }

    // Check if goal coordinates inside map bounds
    if (!m_map.isPosValid (odom_map) || !m_map.isPosValid(goal_map)){
      ROS_WARN ("[ReedsShepp] Path search not possible, coordinates violate map borders.");

      send_empty_path();
      return;
    }

    int8_t startpos_val=m_map.getValue(odom_map.x,odom_map.y);
    for (int dx=-3;dx<3;++dx) {
      for (int dy=-3;dy<3;++dy) {
        m_map.setValue(odom_map.x+dx,odom_map.y+dx,m_threshold_min);
      }
    }

    Curve * curve = m_rs_generator.find_path(odom_map, goal_map, &m_map);
    m_map.setValue(odom_map.x,odom_map.y,startpos_val);

    if(curve && curve->is_valid()){
      publishCurve(curve);
    } else {
      ROS_WARN("[ReedsShepp] Couldn't find a path.");
      send_empty_path();
    }

    if (curve)
      delete curve;

    ROS_INFO("[ReedsShepp] Path generation time: %.4fms", stop_timer());

  } else {
    if(!m_has_odom)
      ROS_ERROR("[ReedsShepp] SEARCH NOT POSSIBLE, BECAUSE ODOM IS MISSING");
    if(!m_has_map)
      ROS_ERROR("[ReedsShepp] SEARCH NOT POSSIBLE, BECAUSE MAP IS MISSING");

    send_empty_path();
  }
}



void ROSReedsSheppPathPlanner::send_empty_path()
{
  m_has_curve = false;

  m_empty_path.header.frame_id = m_publish_frame;

  m_last_path=m_empty_path;

  m_last_weight = 0.0;
  ROS_INFO("last path now size %d",m_last_path.poses.size());
  if (!m_silent_mode)
    m_path_publisher.publish(m_last_path);
}


void ROSReedsSheppPathPlanner::start_timer()
{
  gettimeofday(&m_start_profiling, NULL);
}


double ROSReedsSheppPathPlanner::stop_timer()
{
  gettimeofday(&m_end_profiling, NULL);
  double start_t = m_start_profiling.tv_sec + double(m_start_profiling.tv_usec) / 1e6;
  double end_t = m_end_profiling.tv_sec + double(m_end_profiling.tv_usec) / 1e6;
  double t_diff = end_t - start_t;

  return t_diff * 1000.f;
}


void ROSReedsSheppPathPlanner::send_arrow_marker(int id, Pose2d &pose,
                                                 float scale_x, float scale_y, float scale_z,
                                                 float r, float g, float b, float a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = m_publish_frame;

  marker.ns = "path";
  marker.id = id;

  marker.type = visualization_msgs::Marker::ARROW;

  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose.x;
  marker.pose.position.y = pose.y;
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.lifetime = ros::Duration(15);

  m_marker_publisher.publish(marker);
}


int main(int argc, char** argv)
{
  // subscribe to /goal for goal pose in /odom coordinates
  ros::init(argc,argv, "reeds_shepp_planner");
  ros::NodeHandle n("~");

  ROSReedsSheppPathPlanner node(n);

  ros::Rate rate(200);

  while( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

