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

  computeArcSpan(false);
}

void CircleSegment::set_mode(MODE mode)
{
  m_mode = mode;
}

float CircleSegment::weight(bool ignore_obstacles)
{
#ifdef REED_SHEPP_USE_COST
  m_weight = 0;
#endif
  bool is_free = checkCircle(m_center.x, m_center.y, m_radius, ignore_obstacles);

  if(ignore_obstacles || is_free){
    float cost = (m_direction == CurveSegment::BACKWARD) ? m_cost_backwards : m_cost_forwards;
#ifdef REED_SHEPP_USE_COST
    return cost * m_weight * m_cost_curve;
#else
    return fabs(m_arc_span) * m_radius * cost * m_cost_curve;
#endif
  } else {
    return NOT_FREE;
  }
}


// modified version of midpoint circle algorithm
//   (http://en.wikipedia.org/wiki/Midpoint_circle_algorithm)
// 'cx' and 'cy' denote the offset of the circle centre from the origin.
bool CircleSegment::checkCircle(int cx, int cy, int radius, bool ignore_obstacles)
{
  bool is_free = true;
  int error = -radius;
  int x = radius;
  int y = 0;

  // The following while loop may altered to 'while (x > y)' for a
  // performance benefit, as long as a call to 'test4points' follows
  // the body of the loop. This allows for the elimination of the
  // '(x != y') test in 'test8points', providing a further benefit.
  //
  // For the sake of clarity, this is not shown here.
  while (x >= y)
  {
    is_free &= test8Points(cx, cy, x, y, ignore_obstacles);

    error += y;
    ++y;
    error += y;

    // The following test may be implemented in assembly language in
    // most machines by testing the carry flag after adding 'y' to
    // the value of 'error' in the previous step, since 'error'
    // nominally has a negative value.
    if (error >= 0)
    {
      error -= x;
      --x;
      error -= x;
    }
  }

  return is_free;
}

bool CircleSegment::test8Points(int cx, int cy, int x, int y, bool ignore_obstacles)
{
  bool is_free = test4Points(cx, cy, x, y, ignore_obstacles);
  if (x != y) {
    is_free &= test4Points(cx, cy, y, x, ignore_obstacles);
  }
  return is_free;
}

// The '(x != 0 && y != 0)' test in the last line of this function
// may be omitted for a performance benefit if the radius of the
// circle is known to be non-zero.
bool CircleSegment::test4Points(int cx, int cy, int x, int y, bool ignore_obstacles)
{
  bool is_free = true;
  is_free &= testPixel(cx, cy, cx + x, cy + y, ignore_obstacles);
  if (x != 0) is_free &= testPixel(cx, cy, cx - x, cy + y, ignore_obstacles);
  if (y != 0) is_free &= testPixel(cx, cy, cx + x, cy - y, ignore_obstacles);
  if (x != 0 && y != 0) is_free &= testPixel(cx, cy, cx - x, cy - y, ignore_obstacles);

  return is_free;
}

bool CircleSegment::testPixel(int cx, int cy, int x, int y, bool ignore_obstacles){
  float angle = atan2(y-cy, x-cx);
  if(m_map->isInMap(x, y)){
    bool on_segment = false;

    float start = NORMALIZE(m_angle_start);
    float end = NORMALIZE(start + m_arc_span);

    bool increasing = ((m_orientation == LEFT) ^ (m_direction == BACKWARD)) ^ (m_mode == TOWARDS_POINT);
    bool decreasing = !increasing;

    if(start > end){
      if(decreasing){
        // start > end  &&  angle decreasing  -> no possible overflow
        on_segment = angle <= start && angle >= end;
      } else {
        // start > end  &&  angle increasing  ->  possible overflow
        on_segment = angle >= start || angle <= end;
      }
    } else {
      // start <= end
      if(increasing){
        // start <= end  &&  angle increasing  -> no possible overflow
        on_segment = angle >= start && angle <= end;
      } else {
        // start <= end  &&  angle decreasing  -> possible overflow
        on_segment = angle <= start || angle >= end;
      }
    }


    if(on_segment) {
#ifdef REED_SHEPP_USE_COST
      m_weight += min( (uint8_t)5, m_map->getValue( x, y ));
#endif
      bool free = m_map->isFree(x, y);
      if(m_trace != -1){
        m_map->setValue(x, y, free ? m_trace : std::min(255, m_trace + 50));
      }
      if(ignore_obstacles) return true;
      else
        return free;
    }
  }

  return true;
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

    if ((m_orientation == RIGHT) ^ line_is_reverse)
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

  computeArcSpan(false);
  circle.computeArcSpan(false);

  out_tangent.set_points(tangent_points.first, tangent_points.second);

  return true;
}

void CircleSegment::computeArcSpan(bool is_center_circle)
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

  if(getTangentiaCircleHelper(circle2, circle3, true)) {
    weight_positive = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles);
  }
  if(getTangentiaCircleHelper(circle2, circle3, false)) {
    weight_negative = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles);
  }

  if(weight_negative >= NOT_FREE && weight_positive >= NOT_FREE) {
    return false;

  } else {
    if(weight_positive < weight_negative){
      getTangentiaCircleHelper(circle2, circle3, true);
    } else {
      getTangentiaCircleHelper(circle2, circle3, false);
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

  if(getTangentialDoubleCircleHelper(circle2, circle3, circle4, true)) {
    weight_positive = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles)
        + circle4.weight(ignore_obstacles);
  }
  if(getTangentialDoubleCircleHelper(circle2, circle3, circle4, false)) {
    weight_negative = weight(ignore_obstacles)
        + circle2.weight(ignore_obstacles)
        + circle3.weight(ignore_obstacles)
        + circle4.weight(ignore_obstacles);
  }

  if(weight_negative >= NOT_FREE && weight_positive >= NOT_FREE) {
    return false;

  } else {
    if(weight_positive < weight_negative){
      getTangentialDoubleCircleHelper(circle2, circle3, circle4, true);
    } else {
      getTangentialDoubleCircleHelper(circle2, circle3, circle4, false);
    }
    return true;
  }
}

bool CircleSegment::getTangentiaCircleHelper(CircleSegment &circle2, CircleSegment &circle3,
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
  computeArcSpan(false);

  circle3.m_tangent_angle = (circle3.m_center - circle2.m_center).angle() - DTOR(180);
  circle3.computeArcSpan(false);

  circle2.m_angle_start = (circle2.m_center - m_center).angle() - DTOR(180);
  circle2.m_tangent_angle = (circle2.m_center - circle3.m_center).angle() - DTOR(180);
  circle2.computeArcSpan(true);
  circle2.m_mode = AWAY_FROM_POINT;

  return true;
}



bool CircleSegment::getTangentialDoubleCircleHelper(CircleSegment &circle2, CircleSegment &circle3, CircleSegment &circle4,
                                                        bool choose_positive_solution)
{
  Point2d direction = circle4.center_of_orientation() - this->center_of_orientation();
  double d = direction.distance_to_origin();
  double r = m_radius;
  double two_r = 2 * r;
  double theta = direction.angle();
  //double beta = acos((0.5 * d - r) / two_r);
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
