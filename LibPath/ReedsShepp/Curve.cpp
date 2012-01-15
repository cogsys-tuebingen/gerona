/*
 * Curve.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "../common/MapMath.h"

#include "Curve.h"

#include <iostream>
#include <typeinfo>

using namespace lib_path;

Curve::Curve()
  : m_init(false),
    m_circle_radius(10.0), m_max_waypoint_distance(10.0),
    m_cost_forwards(1.0), m_cost_backwards(1.0), m_cost_curve(1.0), m_cost_straight(1.0),
    m_min_length(NOT_FREE), m_iterating(false), m_output_number(0)
{
  m_min_combo = std::vector<CurveSegment*>();
}

Curve::~Curve() {
  // nothing to do
}

void Curve::test_sequence(std::vector<CurveSegment*> &sequence) {
  if (m_start.distance_to(m_goal) < 2) {
    std::cerr << "***ERROR*** what now?" << std::endl;
    return;
  }
  if (sequence.size() < 3){
    std::cerr << "invalid sequence, length < 3" << std::endl;
    return;
  }

  if(!m_init){
    init_circle_pairs(m_start, m_circle_start);
    init_circle_pairs(m_goal, m_circle_goal);
    m_init = true;
  }

  std::vector<CurveSegment*>::iterator it;

  for(it=sequence.begin(); it!=sequence.end(); ++it){
    (*it)->set_map(m_map);

    (*it)->set_max_distance(m_max_waypoint_distance);

    (*it)->set_cost_backwards(m_cost_backwards);
    (*it)->set_cost_forwards(m_cost_forwards);
    (*it)->set_cost_curve(m_cost_curve);
    (*it)->set_cost_straight(m_cost_straight);

    if(typeid(**it) == typeid(CircleSegment)){
      CircleSegment *c = dynamic_cast<CircleSegment*> (*it);
      c->set_curve_radius(m_circle_radius);
    }
  }


  float length = NOT_FREE;
  if( sequence.size() == 3 )
    length = handle_sequence(sequence[0], sequence[1], sequence[2]);
  else if( sequence.size() == 4 )
    length = handle_sequence(sequence[0], sequence[1], sequence[2], sequence[3]);
  else
    std::cerr << "unknown sequence" << std::endl;

  if(length < m_min_length) {
    m_min_combo.resize(sequence.size());
    for (int i=0;i<sequence.size();++i) {
      if (typeid(*sequence[i])==typeid(CircleSegment)) {
        m_min_combo[i]=new CircleSegment(*((CircleSegment *)sequence[i]));
      } else {
        m_min_combo[i]=new LineSegment(*((LineSegment *)sequence[i]));
      }
    }


    m_min_length = length;
  }
}

double Curve::handle_sequence(CurveSegment *segment1,
                              CurveSegment *segment2,
                              CurveSegment *segment3)
{

  CircleSegment  *start = dynamic_cast<CircleSegment*> (segment1);
  CircleSegment   *goal = dynamic_cast<CircleSegment*> (segment3);

  if(start->orientation() == CircleSegment::LEFT)
    start->set_center_of_rotation(m_circle_start.center_left);
  else
    start->set_center_of_rotation(m_circle_start.center_right);

  if(goal->orientation() == CircleSegment::LEFT)
    goal->set_center_of_rotation(m_circle_goal.center_left);
  else
    goal->set_center_of_rotation(m_circle_goal.center_right);

  start->set_null_direction_angle(m_start.theta);
  goal->set_null_direction_angle(m_goal.theta);

  start->set_mode(CircleSegment::AWAY_FROM_POINT);
  goal->set_mode(CircleSegment::TOWARDS_POINT);

  if(typeid(*segment2) == typeid(CircleSegment)) {
    CircleSegment *center = dynamic_cast<CircleSegment*> (segment2);
    return handle_sequence(start, center, goal);

  } else {
    LineSegment *center = dynamic_cast<LineSegment*> (segment2);
    return handle_sequence(start, center, goal);
  }
}

double Curve::handle_sequence(CurveSegment *segment1,
                              CurveSegment *segment2,
                              CurveSegment *segment3,
                              CurveSegment *segment4)
{

  CircleSegment  *start = dynamic_cast<CircleSegment*> (segment1);
  CircleSegment   *goal = dynamic_cast<CircleSegment*> (segment4);

  if(start->orientation() == CircleSegment::LEFT)
    start->set_center_of_rotation(m_circle_start.center_left);
  else
    start->set_center_of_rotation(m_circle_start.center_right);

  if(goal->orientation() == CircleSegment::LEFT)
    goal->set_center_of_rotation(m_circle_goal.center_left);
  else
    goal->set_center_of_rotation(m_circle_goal.center_right);

  start->set_null_direction_angle(m_start.theta);
  goal->set_null_direction_angle(m_goal.theta);

  start->set_mode(CircleSegment::AWAY_FROM_POINT);
  goal->set_mode(CircleSegment::TOWARDS_POINT);


  if(typeid(*segment2) == typeid(CircleSegment) && typeid(*segment3) == typeid(CircleSegment)) {
    CircleSegment *c1 = dynamic_cast<CircleSegment*> (segment2);
    CircleSegment *c2 = dynamic_cast<CircleSegment*> (segment3);

    return handle_sequence(start, c1, c2, goal);
  }
  std::cerr << "***ERROR*** correct return value?"<< std::cerr << std::endl;
  return NOT_FREE;
}

double Curve::handle_sequence(CircleSegment *circle1, LineSegment *line, CircleSegment *circle2)
{
  bool reverse = line->direction() == CurveSegment::BACKWARD;

  circle1->get_common_tangent(*circle2, *line, reverse);

  float w = circle1->weight() + line->weight() + circle2->weight();

  return w;
}

double Curve::handle_sequence(CircleSegment *circle1, CircleSegment *circle2, CircleSegment *circle3)
{
  if(circle1->get_tangential_circle(*circle2, *circle3) == false){
    return NOT_FREE;

  } else {
    float w = circle1->weight() + circle2->weight() + circle3->weight();
    return w;
  }
}

double Curve::handle_sequence(CircleSegment *start, CircleSegment *circle2, CircleSegment *circle3, CircleSegment *goal)
{
  start->get_tangential_double_circle(*circle2, *circle3, *goal);

  float w = start->weight() + circle2->weight() + circle3->weight() + goal->weight();

  return w;
}

void Curve::init_circle_pairs(Pose2d &next_to, circle_pair &target) {
  float angle = DTOR(90);

  Point2d dir(m_circle_radius, 0.0f);
  dir = dir.rotate(next_to.theta + angle);

  target.center_left = Point2d(next_to.x + dir.x, next_to.y + dir.y);
  target.center_right = Point2d(next_to.x - dir.x, next_to.y - dir.y);
}

bool Curve::is_valid()
{
  return m_min_length < NOT_FREE;
}

double Curve::weight()
{
  double weight=0.0;
  for (unsigned i=0;i<m_min_combo.size();++i) {
    weight+=m_min_combo[i]->weight();
  }
  if (fabs(weight-m_min_length)>1) {
    std::cout << "weight:"<<weight<< " minlength"<<m_min_length<< std::endl;
    std::cout.flush();
  }
  return weight;
}

void Curve::reset_iteration()
{
  if(m_min_combo.size() > 0){
    m_min_combo[0]->reset_iteration();
    m_output_number = 0;
  }
  //*** iterating is never set to false again
  m_iterating = true;
}

bool Curve::has_next()
{
  //*** no asserts in robot code - module should return pathnotfound
  assert(m_iterating);

  if( m_min_length < NOT_FREE &&
      m_iterating &&
      m_output_number < m_min_combo.size() ) {

    return m_min_combo[m_output_number]->has_next() || m_output_number < m_min_combo.size() - 1;

  } else {
    return false;
  }
}

Pose2d Curve::next()
{
  //***
  assert(m_iterating);

  CurveSegment * current_segment = m_min_combo[m_output_number];

  if(current_segment->has_next()) {
    return current_segment->next();

  } else if(has_next()) {
    m_output_number++;
    m_min_combo[m_output_number]->reset_iteration();
    return next();

  } else {
    // ***return undefined value?
    return Pose2d();
  }
}
