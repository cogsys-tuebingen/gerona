/*
 * LineSegment.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */
#include <stdlib.h>
#include "LineSegment.h"

#include "../common/MapMath.h"

using namespace lib_path;

LineSegment::LineSegment(DIRECTION direction)
  : CurveSegment(direction), m_iterating(false), m_output(0)
{
}


void LineSegment::set_points(Point2d start, Point2d end)
{
  m_slope = (end - start).angle();

  m_start.x = start.x;
  m_start.y = start.y;
  m_start.theta = m_slope;

  m_end.x = end.x;
  m_end.y = end.y;
  m_end.theta = m_slope;
}

float LineSegment::weight(bool ignore_obstacles)
{
  bool is_free = true;

  int x0 = m_start.x;
  int y0 = m_start.y;
  int x1 = m_end.x;
  int y1 = m_end.y;

  if(!m_map->isInMap(x0, y0)){
    //std::cout << "line: point (" << x0 << ", " << y0 << ") is outside the map" << std::endl;
    return NOT_FREE;
  }

  int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

#ifdef REED_SHEPP_USE_COST
  double path_cost = 0;
#endif
  for(;;){
    if(m_map->isInMap(x0, y0)){
      bool free = m_map->isFree(x0, y0);
      if(m_trace != -1){
        m_map->setValue(x0, y0, m_trace);
      }
      if(!free && !ignore_obstacles){
        is_free = false;
        break;
      }
#ifdef REED_SHEPP_USE_COST
      path_cost += min( (uint8_t)5, m_map->getValue( x0, y0 ));
#endif
    }

    if (x0==x1 && y0==y1) break;
    e2 = 2*err;
    if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
  }

  if(ignore_obstacles || is_free){
    float cost = (m_direction == CurveSegment::BACKWARD) ? m_cost_backwards : m_cost_forwards;
#ifdef REED_SHEPP_USE_COST
    return cost * path_cost * m_cost_straight;
#else
    return (m_start - m_end).distance_to_origin() * cost * m_cost_straight;
#endif
  } else{
    return NOT_FREE;
  }
}

void LineSegment::reset_iteration()
{
  m_iterating = true;
  m_output = 0;
  m_segments = m_start.distance_to(m_end) / m_max_distance;
}

bool LineSegment::has_next()
{
  assert(m_iterating);

  return m_output < m_segments;
}

Pose2d LineSegment::next()
{
  assert(m_iterating);

  float f = (m_output / (float) m_segments );
  Pose2d ret = m_start + f * (m_end - m_start);

  ret.theta = m_slope;
  if(m_direction == CurveSegment::BACKWARD)
    ret.theta += DTOR(180);

  m_output++;
  return ret;
}

Pose2d LineSegment::start()
{
  return m_start;
}

Pose2d LineSegment::end()
{
  return m_end;
}
