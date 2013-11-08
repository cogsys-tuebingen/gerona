/*
 * CircleFinder.cpp
 *
 *  Created on: Sep 19, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "CircleFinder.h"
#include <limits>
#include <map>
#include <iostream>


namespace {
bool point_smaller_function (Point l,Point r) {
  if(l.x == r.x)
    return l.y < r.y;
  else
    return l.x < r.x;
}

bool point_equal_function (Point l,Point r) {
  return (l.x == r.x) && (l.y == r.y);
}

bool edge_smaller_function (GraphEdge l,GraphEdge r) {
  if(l.x1 == r.x1){
    if(l.x2 == r.x2){
      if(l.y1 == r.y1){
        return l.y2 < r.y2;
      } else {
        return l.y1 < r.y1;
      }
    } else {
      return l.x2 < r.x2;
    }
  } else {
    return l.x1 < r.x1;
  }
}

bool edge_equal_function (GraphEdge l,GraphEdge r) {
  return (l.x1 == r.x1) && (l.y1 == r.y1) && (l.x2 == r.x2) && (l.y2 == r.y2);
}

struct cmp_point {
  bool operator() (const Point& lhs, const Point& rhs) const
  {
    if (lhs.x != rhs.x)
      return lhs.x < rhs.x;
    else
      return lhs.y < rhs.y;
  }
};

}


CircleFinder::CircleFinder(int min_x, int max_x, int min_y, int max_y, int min_dist)
  : m_min_x(min_x), m_max_x(max_x), m_min_y(min_y), m_max_y(max_y), m_min_dist(min_dist)
{
}
void CircleFinder::run(std::vector<Point> &obstacles, bool use_naiive, bool use_unique)
{
  //std::cout << "started search on " << obstacles.size() << " points" << std::endl;

  if(use_naiive)
    run_naiive(obstacles);
  else
    run_map(obstacles, use_unique);
}

void CircleFinder::run_map(std::vector<Point> &obstacles, bool use_unique)
{
  profiler.start();

  m_obstacles = obstacles;

  if(m_obstacles.size() <= 2)
    return;

  // clear node and edge lists
  m_voronoi_nodes.clear();
  m_voronoi_edges.clear();

  // generate the voronoi diagram
  VoronoiDiagramGenerator vdg;
  vdg.generateVoronoi(m_obstacles, m_min_x, m_max_x, m_min_y, m_max_y, m_min_dist);
  vdg.resetIterator();

  std::map <Point, std::vector<Point>, cmp_point > node_to_sites;
  std::map <Point, std::vector<Point>, cmp_point > ::iterator map_it;

  Point from, to, p1, p2;
  while(vdg.getNext(from, to, p1, p2))
  {
    m_voronoi_nodes.push_back(from);
    m_voronoi_nodes.push_back(to);

    node_to_sites[from].push_back(p1);
    node_to_sites[from].push_back(p2);

    node_to_sites[to].push_back(p1);
    node_to_sites[to].push_back(p2);

    // add the edge to the edge list
    GraphEdge g;
    g.x1 = from.x;
    g.x2 = to.x;
    g.y1 = from.y;
    g.y2 = to.y;
    g.sites[0] = p1;
    g.sites[1] = p2;
    g.next = NULL;
    m_voronoi_edges.push_back(g);
  }

  if(use_unique){
    // make points and edges unique
    // seems to have a negative impact on the execution speed
    {
      std::vector<Point>::iterator unique_it;
      std::sort(m_voronoi_nodes.begin(), m_voronoi_nodes.end(), point_smaller_function);
      unique_it = std::unique(m_voronoi_nodes.begin(), m_voronoi_nodes.end(), point_equal_function);
      m_voronoi_nodes.resize( unique_it - m_voronoi_nodes.begin() );
    }
    {
      std::vector<GraphEdge>::iterator unique_it;
      std::sort(m_voronoi_edges.begin(), m_voronoi_edges.end(), edge_smaller_function);
      unique_it = std::unique(m_voronoi_edges.begin(), m_voronoi_edges.end(), edge_equal_function);
      m_voronoi_edges.resize( unique_it - m_voronoi_edges.begin() );
    }
  }
  if (node_to_sites.empty()) {
    m_radius = 0;
    m_center.x=m_center.y = 0.0;
    std::cout << "***ERROR*** voronoi diagram empty in circlefinder"<<std::endl;
    return;
  }

  float max_obstacle_dist_squared = 0;
  Point voronoi_node_with_max_dist=node_to_sites.begin()->first;
  std::vector<Point>::iterator it_inner;

  // for each voronoi node, check the distances to all neighboring cell's obstacle and to the borders
  for(map_it = node_to_sites.begin(); map_it != node_to_sites.end(); ++map_it){
    const Point &voronoi_node = map_it->first;
    std::vector<Point> neighbors = map_it->second;

    float left  = abs(voronoi_node.x - m_min_x);
    float right = abs(voronoi_node.x - m_max_x);
    float up    = abs(voronoi_node.y - m_min_y);
    float down  = abs(voronoi_node.y - m_max_y);
    float border_dist = std::min(std::min(std::min(left, right), up), down);

    float nearest_obstacle_dist_squared = std::numeric_limits<float>::max();
    Point nearest_obstacle;

    // iterate over all neighboring obstacles to find the closest
    for(it_inner = neighbors.begin(); it_inner != neighbors.end(); ++it_inner){
      const Point &obstacle = (*it_inner);
      float dist_squared = pow(voronoi_node.x - obstacle.x, 2) + pow(voronoi_node.y - obstacle.y, 2);
      if(dist_squared < nearest_obstacle_dist_squared){
        nearest_obstacle_dist_squared = dist_squared;
        nearest_obstacle = voronoi_node;
      }
    }

    if(nearest_obstacle_dist_squared > max_obstacle_dist_squared){
      // the closest obstacle is farther away from the current voronoi node, than anyone before

      if(nearest_obstacle_dist_squared > pow(border_dist, 2)) {
        // the current voronoi node is closer to the border than to the obstacle
        // clamp the distance to the wall distance
        if(border_dist > max_obstacle_dist_squared){
          // the clamped distance is still an improvement for the maximum distance
          max_obstacle_dist_squared = pow(border_dist, 2);
          voronoi_node_with_max_dist = voronoi_node;
        }

      } else {
        // the wall distance is smaller than the distance to the next obstacle
        max_obstacle_dist_squared = nearest_obstacle_dist_squared;
        voronoi_node_with_max_dist = voronoi_node;
      }
    }
  }

  m_radius = sqrt(max_obstacle_dist_squared);
  m_center = voronoi_node_with_max_dist;
  m_time = profiler.stop();
}

void CircleFinder::run_naiive(std::vector<Point> &obstacles)
{
  profiler.start();

  m_obstacles = obstacles;

  if(m_obstacles.size() <= 2)
    return;

  // clear node and edge lists
  m_voronoi_nodes.clear();
  m_voronoi_edges.clear();

  // generate the voronoi diagram
  VoronoiDiagramGenerator vdg;
  vdg.generateVoronoi(m_obstacles, m_min_x, m_max_x, m_min_y, m_max_y, m_min_dist);
  vdg.resetIterator();

  // iterate over all voronoi edges
  float x1,y1,x2,y2;
  while(vdg.getNext(x1,y1,x2,y2))
  {
    // add both endpoints to the node list
    Point from, to;
    from.x = x1; from.y = y1;
    to.x = x2; to.y = y2;

    m_voronoi_nodes.push_back(from);
    m_voronoi_nodes.push_back(to);

    // add the edge to the edge list
    GraphEdge g;
    g.x1 = x1;
    g.x2 = x2;
    g.y1 = y1;
    g.y2 = y2;
    g.next = NULL;
    m_voronoi_edges.push_back(g);
  }


  // iterators for the two lists
  std::vector<Point>::iterator point_it;
  std::vector<Point>::iterator it_inner;

  float max_obstacle_dist_squared = 0;
  Point voronoi_node_with_max_dist =m_voronoi_nodes.front();


  // find the node with the largest distance to an obstacle

  // foreach voronoi node find the nearest obstacle
  for(point_it = m_voronoi_nodes.begin(); point_it != m_voronoi_nodes.end(); ++point_it){
    const Point &voronoi_node = (*point_it);

    // check 4 borders
    float left  = abs(voronoi_node.x - m_min_x);
    float right = abs(voronoi_node.x - m_max_x);
    float up    = abs(voronoi_node.y - m_min_y);
    float down  = abs(voronoi_node.y - m_max_y);
    float border_dist = std::min(std::min(std::min(left, right), up), down);

    float nearest_obstacle_dist_squared = std::numeric_limits<float>::max();
    Point nearest_obstacle;

    // iterate over all obstacles to find the closest
    for(it_inner = m_obstacles.begin(); it_inner != m_obstacles.end(); ++it_inner){
      const Point &obstacle = (*it_inner);
      float dist_squared = pow(voronoi_node.x - obstacle.x, 2) + pow(voronoi_node.y - obstacle.y, 2);
      if(dist_squared < nearest_obstacle_dist_squared){
        nearest_obstacle_dist_squared = dist_squared;
        nearest_obstacle.x = voronoi_node.x;
        nearest_obstacle.y = voronoi_node.y;
      }
    }

    if(nearest_obstacle_dist_squared > max_obstacle_dist_squared){
      // the closest obstacle is farther away from the current voronoi node, than anyone before

      if(nearest_obstacle_dist_squared > pow(border_dist, 2)) {
        // the current voronoi node is closer to the border than to the obstacle
        // clamp the distance to the wall distance
        if(border_dist > max_obstacle_dist_squared){
          // the clamped distance is still an improvement for the maximum distance
          max_obstacle_dist_squared = pow(border_dist, 2);
          voronoi_node_with_max_dist = voronoi_node;
        }

      } else {
        // the wall distance is smaller than the distance to the next obstacle
        max_obstacle_dist_squared = nearest_obstacle_dist_squared;
        voronoi_node_with_max_dist = voronoi_node;
      }
    }
  }

  m_radius = sqrt(max_obstacle_dist_squared);
  m_center = voronoi_node_with_max_dist;
  m_time = profiler.stop();
}

bool CircleFinder::valid()
{
  return m_obstacles.size() >= 2;
}

std::vector<Point> CircleFinder::get_voronoi_points()
{
  return m_voronoi_nodes;
}
std::vector<GraphEdge> CircleFinder::get_voronoi_edges()
{
  return m_voronoi_edges;
}

Point CircleFinder::get_center()
{
  return m_center;
}

double CircleFinder::get_radius()
{
  return m_radius;
}

double CircleFinder::get_time_needed()
{
  return m_time;
}

