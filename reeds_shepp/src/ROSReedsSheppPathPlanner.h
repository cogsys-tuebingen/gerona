/*
 * ROSReedsSheppPathPlanner.h
 *
 *  Created on: Aug 19, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef REEDSSHEPPPATHPLANNER_H
#define REEDSSHEPPPATHPLANNER_H

#include <utils/LibPath/ReedsShepp/CurveGenerator.h>
#include <utils/LibPath/common/Map.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <string>

class ROSReedsSheppPathPlanner
{
public:
  ROSReedsSheppPathPlanner(const ros::NodeHandle& n, const bool silent_mode = false);

  /**
   * Callbacks
   */
  void update_goal (const geometry_msgs::PoseStampedConstPtr &goal);
  void update_ring_goal (const geometry_msgs::PointConstPtr &ring_goal);
  void update_odometry (const nav_msgs::OdometryConstPtr &odom);
  void update_map (const nav_msgs::OccupancyGridConstPtr &map);

  /**
   * Calculate the shortest path and publish it
   */
  void calculate();



private:
  void send_arrow_marker(int id, lib_path::Pose2d &pose,
                         float scale_x, float scale_y, float scale_z,
                         float r, float g, float b, float a);

  void send_empty_path();
  void publishCurve (lib_path::Curve * curve);

  void start_timer();
  double stop_timer();


  // Initialization functions during constructor
  void init_parameters();
  void generate_rs_patterns();
  void set_generator_parameters();
  void init_subscribers();
  void init_publishers();

  // Member variables
  ros::NodeHandle m_node_handle;

  lib_path::CurveGenerator m_rs_generator;
  double m_circle_radius;
  double m_max_waypoint_distance;
  double m_cost_backwards;
  double m_cost_forwards;
  double m_cost_curve;
  double m_cost_straight;

  int m_threshold_min;
  int m_threshold_max;

  bool m_silent_mode;


  bool m_has_curve;

  bool m_has_goal;
  bool m_has_odom;
  bool m_has_map;

  double m_last_weight;
  nav_msgs::Path m_last_path;
  nav_msgs::Path m_empty_path;
  std::string m_map_topic;
  std::string m_goal_topic;
  std::string m_ring_goal_topic;
  std::string m_publish_frame;

  ros::Subscriber m_goal_pos_subscriber;
  ros::Subscriber m_ring_goal_subscriber;
  ros::Subscriber m_odom_subscriber;
  ros::Subscriber m_map_subscriber;

  ros::Publisher m_path_publisher;
  ros::Publisher m_marker_publisher;

  tf::TransformListener listener_;

  lib_path::Pose2d m_goal_world;
  lib_path::Pose2d m_odom_world;

  lib_path::MapInfo m_map;

  struct timeval m_start_profiling, m_end_profiling;
};

#endif // REEDSSHEPPPATHPLANNER_H
