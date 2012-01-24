/*
 * lec_node.cpp
 *
 *  Created on: Sep 19, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include <ros/ros.h>
#include "CircleFinder.h"
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include "Timer.h"

ros::Publisher m_marker_publisher;
ros::Publisher m_circle_publisher;
bool publish_debug_marker = false;
bool publish_verbose_debug_marker = false;
double min_distance = sqrt(2); /*meters*/
double grid_spacing = 1.0; /*meters*/

#define PROFILE_ALL 0
#define PROFILE_INNER 1
Timer profiler[2];

int threshold_for_obstacles = 1;

inline bool is_obstacle(int v){
  return v > threshold_for_obstacles || v < 0;
}
inline bool is_free(int v){
  return v <= threshold_for_obstacles && v >= 0;
}

void update_map(const nav_msgs::OccupancyGridConstPtr &map)
{
  profiler[PROFILE_ALL].start();

  int    w   = map->info.width;
  int    h   = map->info.height;
  double res = map->info.resolution;
  double ox  = map->info.origin.position.x;
  double oy  = map->info.origin.position.y;

  int count = 0;

  visualization_msgs::Marker verbose_marker;
  if(publish_verbose_debug_marker){
    verbose_marker.header.frame_id = "/map";
    verbose_marker.ns = "freespace_obstacles";
    verbose_marker.id = 0;
    verbose_marker.type = visualization_msgs::Marker::POINTS;
    verbose_marker.action = visualization_msgs::Marker::ADD;
    verbose_marker.scale.x = 0.2;
    verbose_marker.scale.y = 0.2;
    verbose_marker.scale.z = 0.2;
    verbose_marker.color.r = 1;
    verbose_marker.color.g = 0;
    verbose_marker.color.b = 0;
    verbose_marker.color.a = 1;
    verbose_marker.lifetime = ros::Duration(0);
  }

  int grid = (int) (grid_spacing / res);

  // make a list of all obstacles
  std::vector<Point> points;
  for(int y=0; y<h; y++){
    for(int x=0; x<w; x++){
      int v = map->data[y*w+x];

      if(is_obstacle(v)){
        count++;

        bool has_free_neighbor =
            (x+1 < w && is_free(map->data[y*w+x+1]))   ||
            (x   > 0 && is_free(map->data[y*w+x-1]))   ||
            (y+1 < h && is_free(map->data[(y+1)*w+x])) ||
            (y   > 0 && is_free(map->data[(y-1)*w+x]));

        bool is_on_grid = (x % grid == 0 && y % grid == 0);

        if(has_free_neighbor || is_on_grid){
          Point p;
          p.x = ox + x * res;
          p.y = oy + y * res;
          points.push_back(p);

          if(publish_verbose_debug_marker){
            geometry_msgs::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.05;
            verbose_marker.points.push_back(pt);
          }
        }
      }
    }
  }

  if(publish_verbose_debug_marker)
    m_marker_publisher.publish(verbose_marker);

  ROS_DEBUG_STREAM("reduced the no. of points from " << count << " to " << points.size());

  bool use_naiive_method = false;
  bool make_obstacles_unique = false;

  // start the search
  profiler[PROFILE_INNER].start();
  CircleFinder finder((int) ox,(int) (w+ox),(int) (oy), (int) (h+oy), min_distance);
  finder.run(points, use_naiive_method, make_obstacles_unique);

  ROS_DEBUG_STREAM("freespace search took " << profiler[PROFILE_INNER].stop() << "ms");

  if(finder.valid()){
    // voronoi search was successful
    Point center = finder.get_center();
    double radius = finder.get_radius();

    // visualize voronoi edges
    visualization_msgs::Marker line_list;
    std::vector<GraphEdge> edge = finder.get_voronoi_edges();
    std::vector<GraphEdge>::iterator it;
    for(it = edge.begin(); it != edge.end(); ++it){
      geometry_msgs::Point from, to;
      from.x = it->x1;
      from.y = it->y1;
      to.x = it->x2;
      to.y = it->y2;
      line_list.points.push_back(from);
      line_list.points.push_back(to);
    }

    // publish the circle
    geometry_msgs::Point circle;
    circle.x = center.x;
    circle.y = center.y;
    circle.z = radius;
    m_circle_publisher.publish(circle);

    // visualize circle
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.ns = "freespace_circle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = 0.05;
    marker.scale.x = radius*2;
    marker.scale.y = radius*2;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 1;
    marker.color.b = 0.5;
    marker.color.a = 0.75;
    marker.lifetime = ros::Duration(0);
    m_marker_publisher.publish(marker);

    // publish voronoi edges, if enabled
    if(publish_debug_marker){
      line_list.header.frame_id = "/map";
      line_list.ns = "freespace_voronoi";
      line_list.id = 0;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.scale.x = 0.1;
      line_list.scale.y = 0.1;
      line_list.scale.z = 0.1;
      line_list.color.r = 0.5;
      line_list.color.g = 0.5;
      line_list.color.b = 1;
      line_list.color.a = 0.75;
      line_list.lifetime = ros::Duration(0);
      m_marker_publisher.publish(line_list);
    }
  }

  ROS_DEBUG_STREAM("whole method search took " << profiler[PROFILE_ALL].stop() << "ms");
}

int main(int argc, char** argv)
{
  ros::init(argc,argv, "largest_free_circle_finder");
  ros::NodeHandle n("~");

  std::string map_topic;
  n.param<std::string> ("map_topic", map_topic, "/map");
  n.param("grid_spacing", grid_spacing, 1.0);
  n.param("publish_debug_marker", publish_debug_marker, false);
  n.param("publish_verbose_debug_marker", publish_verbose_debug_marker, false);

  min_distance = 2 * grid_spacing;

  if(publish_verbose_debug_marker)
    publish_debug_marker = true;

  ros::Subscriber m_map_subscriber = n.subscribe<nav_msgs::OccupancyGrid> (map_topic, 2, &update_map);
  publish_debug_marker = true;

  m_marker_publisher = n.advertise<visualization_msgs::Marker> ("/freespace", 200);

  // the circle is published as a point, where the z coordinate represents the circle's radius
  m_circle_publisher = n.advertise<geometry_msgs::Point> ("/largest_circle", 200);

  ros::Rate rate(200);

  while( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
