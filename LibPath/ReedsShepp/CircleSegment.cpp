/*
 * CircleSegment.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "CircleSegment.h"

#include <iostream>

using namespace lib_path;

CircleSegment::CircleSegment(ORIENTATION orientation, DIRECTION direction)
  : CurveSegment(direction), m_orientation(orientation), m_angle_start(0), m_tangent_angle(0), m_iterating(false), m_output(0)
{
}
/*
CircleSegment::CircleSegment(const CircleSegment *src)
{

  m_orientation=src->m_orientation;
  m_mode=src->m_mode;

  m_center=src->m_center;
  m_radius=;

  m_angle_start;
  m_tangent_angle;

  m_arc_span;
  m_stepsize;

  m_iterating;
  m_output;
  m_steps;
  m_jump_over;


*/
CircleSegment::ORIENTATION CircleSegment::orientation()
{
  return m_orientation;
}

void CircleSegment::set_center_of_rotation(Point2d &center)
{
  m_center = center;
}

Point2d CircleSegment::center_of_orientation()
{
  return m_center;
}

double CircleSegment::radius()
{
  return m_radius;
}

void CircleSegment::set_curve_radius(double radius)
{
  m_radius = radius;
  m_steps = m_radius * 2 * M_PI;
}

void CircleSegment::set_null_direction_angle(double angle)
{
  m_angle_start = angle + DTOR(90);
  if(m_orientation == CircleSegment::LEFT)
    m_angle_start += DTOR(180);
}

void CircleSegment::set_end_angle(double angle)
{
  m_tangent_angle = angle + DTOR(90);
  if(m_orientation == CircleSegment::LEFT)
    m_tangent_angle += DTOR(180);

  compute_arc_span(false);
}

void CircleSegment::set_mode(MODE mode)
{
  m_mode = mode;
}

float CircleSegment::weight(bool ignore_obstacles)
{
  //ignore_obstacles = true;
  bool is_free = true;

  Point2d ray(m_radius, 0);
  for (int step = 0; step <= m_steps; step++) {
    float angle = m_angle_start + m_stepsize * step;

    Point2d point_on_map = m_center + ray.rotate(angle);

    if(m_map->isInMap((int)point_on_map.x, (int)point_on_map.y)){

      if(!m_map->isFree((int) point_on_map.x, (int) point_on_map.y)){
        is_free = false;
        break;
      }
    } else {
      //std::cout << "circle: point (" << (int) point_on_map.x << ", " << (int) point_on_map.y << ") is outside the map" << std::endl;
      is_free = false;
      break;
    }
  }

  if(ignore_obstacles || is_free){
    float cost = (m_direction == CurveSegment::BACKWARD) ? m_cost_backwards : m_cost_forwards;
    return fabs(m_arc_span) * m_radius * cost * m_cost_curve;
  } else {
    return NOT_FREE;
  }
}


bool CircleSegment::get_common_tangent(CircleSegment &circle, LineSegment &out_tangent, bool line_is_reverse)
{
  std::pair<Point2d, Point2d> tangent_points;
  Point2d &p = tangent_points.first;
  Point2d &q = tangent_points.second;

  Point2d start = m_center;
  Point2d goal = circle.m_center;

  Point2d direction = goal - start;
  float distance = direction.distance_to_origin();

  float orientation = direction.angle();

  if (m_orientation == circle.m_orientation) {
    m_tangent_angle = orientation + DTOR((m_orientation == CircleSegment::LEFT) ? -90 : 90);
    if(line_is_reverse)
      m_tangent_angle -= DTOR(180);

    circle.m_tangent_angle = m_tangent_angle;

    Point2d offset = direction.ortho() / distance * m_radius;

    if (m_orientation == RIGHT ^ line_is_reverse)
      offset = offset * -1;

    p = start + offset;
    q = goal + offset;

  } else {
    float gamma = acos(2.0 * m_radius / distance);

    if(!( (m_direction == CurveSegment::FORWARD && line_is_reverse) ||   line_is_reverse)){
      m_tangent_angle = orientation + ((m_orientation == RIGHT) ? gamma : -gamma);
    } else {
      m_tangent_angle = orientation + ((m_orientation == LEFT) ?  gamma : -gamma);
    }
    circle.m_tangent_angle = m_tangent_angle + DTOR(180);

    Point2d offset(m_radius, 0);
    offset = offset.rotate(m_tangent_angle);

    p = start + offset;
    q = goal - offset;
  }

  compute_arc_span(false);
  circle.compute_arc_span(false);

  out_tangent.set_points(tangent_points.first, tangent_points.second);

  return true;
}

void CircleSegment::compute_arc_span(bool is_center_circle)
{
  float arc_angle = NORMALIZE_0_2PI(m_tangent_angle - m_angle_start );

  if(is_center_circle){
    if(m_orientation == RIGHT && m_direction == FORWARD)
      arc_angle -= 2 * M_PI;
    if(m_orientation == LEFT && m_direction == BACKWARD)
      arc_angle -= 2 * M_PI;

  } else {
    if(m_orientation == RIGHT && m_direction == BACKWARD && m_mode == TOWARDS_POINT)
      arc_angle -= 2 * M_PI;
    else if(m_orientation == RIGHT && m_direction == FORWARD && m_mode == AWAY_FROM_POINT)
      arc_angle -= 2 * M_PI;
    else if(m_orientation == LEFT && m_direction == FORWARD && m_mode == TOWARDS_POINT)
      arc_angle -= 2 * M_PI;
    else if(m_orientation == LEFT && m_direction == BACKWARD && m_mode == AWAY_FROM_POINT)
      arc_angle -= 2 * M_PI;
  }

  m_arc_span = arc_angle;
  m_stepsize = m_arc_span / m_steps;
}

bool CircleSegment::get_tangential_circle(CircleSegment &circle2, CircleSegment &circle3,
                                          bool ignore_obstacles)
{
  Point2d &c1 = m_center;
  Point2d &c3 = circle3.m_center;

  if((c1 - c3).distance_to_origin() < 2 || (c1 - c3).distance_to_origin() > 4 * m_radius)
    return false;

  float weight_positive = NOT_FREE;
  float weight_negative = NOT_FREE;

  if(get_tangential_circle_helper(circle2, circle3, true)) {
    weight_positive = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles);
  }
  if(get_tangential_circle_helper(circle2, circle3, false)) {
    weight_negative = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles);
  }

  if(weight_negative >= NOT_FREE && weight_positive >= NOT_FREE) {
    return false;

  } else {
    if(weight_positive < weight_negative){
      get_tangential_circle_helper(circle2, circle3, true);
    } else {
      get_tangential_circle_helper(circle2, circle3, false);
    }
    return true;
  }
}

bool CircleSegment::get_tangential_double_circle(CircleSegment &circle2, CircleSegment &circle3, CircleSegment &circle4,
                                                 bool ignore_obstacles)
{

  Point2d &c1 = m_center;
  Point2d &c4 = circle4.m_center;

  if((c1 - c4).distance_to_origin() < 2 || (c1 - c4).distance_to_origin() > 6 * m_radius)
    return false;

  float weight_positive = NOT_FREE;
  float weight_negative = NOT_FREE;

  if(get_tangential_double_circle_helper(circle2, circle3, circle4, true)) {
    weight_positive = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles)
        + circle4.weight(ignore_obstacles);
  }
  if(get_tangential_double_circle_helper(circle2, circle3, circle4, false)) {
    weight_negative = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles)
        + circle4.weight(ignore_obstacles);
  }

  if(weight_negative >= NOT_FREE && weight_positive >= NOT_FREE) {
    return false;

  } else {
    if(weight_positive < weight_negative){
      get_tangential_double_circle_helper(circle2, circle3, circle4, true);
    } else {
      get_tangential_double_circle_helper(circle2, circle3, circle4, false);
    }
    return true;
  }
}

bool CircleSegment::get_tangential_circle_helper(CircleSegment &circle2, CircleSegment &circle3,
                                                 bool choose_positive_solution)
{
  Point2d &c1 = m_center;
  Point2d &c3 = circle3.m_center;

  float m = (c1.x - c3.x) / (c3.y - c1.y);
  float n = (pow(c3.y, 2.0)-pow(c1.y, 2.0)+pow(c3.x, 2.0)-pow(c1.x, 2.0)) / (2.0 * (c3.y - c1.y));

  float p = (2.0 * (-c1.x + m * (n - c1.y))) / (1.0 + pow(m, 2.0));
  float q = (pow(c1.x, 2.0) + pow(n-c1.y, 2.0) - pow(2*m_radius,2.0)) / (1.0 + pow(m, 2.0));

  float sqrt_ = sqrt(pow(p/2.0, 2) - q);

  float xsp = - p/2.0 + sqrt_;
  float xsn = - p/2.0 - sqrt_;

  float ysp = m * xsp + n;
  float ysn = m * xsn + n;

  Point2d center_positive(xsp, ysp);
  Point2d center_negative(xsn, ysn);

  if(choose_positive_solution){
    circle2.m_center = center_positive;
  } else {
    circle2.m_center = center_negative;
  }

  m_tangent_angle = (m_center - circle2.m_center).angle() - DTOR(180);
  compute_arc_span(false);

  circle3.m_tangent_angle = (circle3.m_center - circle2.m_center).angle() - DTOR(180);
  circle3.compute_arc_span(false);

  circle2.m_angle_start = (circle2.m_center - m_center).angle() - DTOR(180);
  circle2.m_tangent_angle = (circle2.m_center - circle3.m_center).angle() - DTOR(180);
  circle2.compute_arc_span(true);
  circle2.m_mode = AWAY_FROM_POINT;

  return true;
}



bool CircleSegment::get_tangential_double_circle_helper(CircleSegment &circle2, CircleSegment &circle3, CircleSegment &circle4,
                                                        bool choose_positive_solution)
{
  Point2d direction = circle4.center_of_orientation() - this->center_of_orientation();
  double d = direction.distance_to_origin();
  double r = m_radius;
  double two_r = 2 * r;
  double theta = direction.angle();
  double beta = acos((0.5 * d - r) / two_r);
  double q = (d-two_r) / 2.0;
  double delta = acos(q/two_r);
  if(!choose_positive_solution)
    delta = -delta;

  // compute center for inner circles
  Point2d offset(m_radius*2, 0);
  Point2d offset2(m_radius*2, 0);
  offset = offset.rotate(delta+theta);
  offset2 = offset2.rotate(theta);

  Point2d center2 = this->center_of_orientation() + offset;
  Point2d center3 = center2 + offset2;

  circle2.set_center_of_rotation(center2);
  circle3.set_center_of_rotation(center3);

  // compute and set arc begin for in(ner circles
  double circle_2_offset = (circle2.m_orientation == CircleSegment::LEFT) ? -DTOR(90) : DTOR(90);
  circle2.set_end_angle(delta+theta+circle_2_offset);
  circle2.set_null_direction_angle(theta-circle_2_offset);

  double circle_3_offset = (circle3.m_orientation == CircleSegment::RIGHT) ? -DTOR(90) : DTOR(90);
  circle3.set_null_direction_angle(-delta+theta+circle_3_offset);
  circle3.set_end_angle(theta-circle_3_offset);

  // compute and set arc end for inner circles
  if((circle2.m_direction == CurveSegment::BACKWARD) ^ (circle2.m_direction == CircleSegment::FORWARD))
    circle2.set_mode(CircleSegment::TOWARDS_POINT);
  else
    circle2.set_mode(CircleSegment::AWAY_FROM_POINT);

  if((circle3.m_direction == CurveSegment::FORWARD) ^ (circle3.m_direction == CircleSegment::BACKWARD))
    circle3.set_mode(CircleSegment::TOWARDS_POINT);
  else
    circle3.set_mode(CircleSegment::AWAY_FROM_POINT);

  // start and end circle
  if(this->m_orientation == CircleSegment::LEFT)
    this->set_end_angle(delta+theta+DTOR(90));
  else
    this->set_end_angle(delta+theta+DTOR(-90));

  if(circle4.m_orientation == CircleSegment::LEFT)
    circle4.set_end_angle(-delta+theta+DTOR(-90));
  else
    circle4.set_end_angle(-delta+theta+DTOR(90));

  return true;
}

void CircleSegment::reset_iteration()
{
  m_iterating = true;
  m_output = 0;

  float no_of_points_in_output = fabs(m_arc_span * m_radius) / m_max_distance;
  m_jump_over = std::max(1, (int) (m_steps / no_of_points_in_output));
}

bool CircleSegment::has_next()
{
  assert(m_iterating);

  return m_output <= m_steps;
}

Pose2d CircleSegment::next()
{
  assert(m_iterating);

  Point2d ray(m_radius, 0);

  float angle;
  if(m_mode == AWAY_FROM_POINT){
    if(m_arc_span > 0)
      angle = std::min( m_angle_start + m_arc_span,
                        m_angle_start +              m_stepsize * m_output);
    else
      angle = std::max( m_angle_start + m_arc_span,
                        m_angle_start +              m_stepsize * m_output);

  } else {
    if(m_arc_span > 0)
      angle = std::max( m_angle_start,
                        m_angle_start + m_arc_span - m_stepsize * m_output);
    else
      angle = std::min( m_angle_start,
                        m_angle_start + m_arc_span - m_stepsize * m_output);
  }
  Point2d pt = m_center + ray.rotate(angle);

  Pose2d ret;
  ret.x = pt.x;
  ret.y = pt.y;
  ret.theta = angle + ((m_orientation == RIGHT) ? DTOR(-90) : DTOR(90));

  m_output+=m_jump_over;

  return ret;
}
