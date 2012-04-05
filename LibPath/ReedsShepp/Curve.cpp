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
  : m_init(false), m_ignore_obstacles(false),
    m_circle_radius(10.0), m_max_waypoint_distance(10.0),
    m_use_map_cost(false), m_min_cell_cost(10),
    m_cost_forwards(1.0), m_cost_backwards(1.0), m_cost_curve(1.0), m_cost_straight(1.0),
    m_min_length(NOT_FREE), m_iterating(false), m_output_number(0), m_trace(-1)
{
  m_min_combo = std::vector<CurveSegment*>();
}

Curve::Curve(const Curve &c)
  : m_init(c.m_init), m_ignore_obstacles(c.m_ignore_obstacles),
    m_circle_radius(c.m_circle_radius), m_max_waypoint_distance(c.m_max_waypoint_distance),
    m_use_map_cost(c.m_use_map_cost), m_min_cell_cost(c.m_min_cell_cost),
    m_cost_forwards(c.m_cost_forwards), m_cost_backwards(c.m_cost_backwards),
    m_cost_curve(c.m_cost_curve), m_cost_straight(c.m_cost_straight),
    m_min_length(c.m_min_length),
    m_circle_start(c.m_circle_start), m_circle_goal(c.m_circle_goal),
    m_start(c.m_start), m_goal(c.m_goal),
    m_map(c.m_map),
    m_iterating(c.m_iterating), m_output_number(c.m_output_number),
    m_trace(c.m_trace)
{
  m_min_combo = std::vector<CurveSegment*>();
  m_min_combo.reserve(c.m_min_combo.size());
  std::vector<CurveSegment*>::const_iterator it = c.m_min_combo.begin();

  for(; it != c.m_min_combo.end(); ++it){
    CurveSegment * current = *it;
    CircleSegment * circle = dynamic_cast<CircleSegment*>(current);
    LineSegment * line = dynamic_cast<LineSegment*>(current);

    if(circle != NULL){
      m_min_combo.push_back(new CircleSegment(*circle));
    } else if(line != NULL) {
      m_min_combo.push_back(new LineSegment(*line));
    } else {
      std::cerr << "unknown curve segment" << std::endl;
    }
  }
}

Curve::~Curve() {
  std::vector<CurveSegment*>::const_iterator it = m_min_combo.begin();

  for(; it != m_min_combo.end(); ++it){
    delete *it;
  }
}

void Curve::test_sequence(std::vector<CurveSegment*> &sequence) {
  if (m_start.distance_to(m_goal) < 2) {
    // TODO: robot is 2 pixels away from the target, what to do?
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

    (*it)->set_use_map_cost(m_use_map_cost);
    (*it)->set_min_cell_cost(m_min_cell_cost);
    (*it)->set_cost_backwards(m_cost_backwards);
    (*it)->set_cost_forwards(m_cost_forwards);
    (*it)->set_cost_curve(m_cost_curve);
    (*it)->set_cost_straight(m_cost_straight);

    (*it)->set_trace(m_trace);

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

  if(length < NOT_FREE) {
    //std::cout << "found a curve" << std::endl;
  }

  if(length < m_min_length) {
    m_min_combo.resize(sequence.size());
    for (unsigned i=0;i<sequence.size();++i) {
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

  std::cerr << "unknown sequence" << std::endl;
  return NOT_FREE;
}

double Curve::handle_sequence(CircleSegment *circle1, LineSegment *line, CircleSegment *circle2)
{
  bool reverse = line->direction() == CurveSegment::BACKWARD;

  circle1->get_common_tangent(*circle2, *line, reverse);

  float w = circle1->weight(m_ignore_obstacles)
      + line->weight(m_ignore_obstacles)
      + circle2->weight(m_ignore_obstacles);

  return w;
}

double Curve::handle_sequence(CircleSegment *circle1, CircleSegment *circle2, CircleSegment *circle3)
{
  if(circle1->get_tangential_circle(*circle2, *circle3, m_ignore_obstacles) == false){
    return NOT_FREE;

  } else {
    float w = circle1->weight(m_ignore_obstacles)
        + circle2->weight(m_ignore_obstacles)
        + circle3->weight(m_ignore_obstacles);
    return w;
  }
}

double Curve::handle_sequence(CircleSegment *start, CircleSegment *circle2, CircleSegment *circle3, CircleSegment *goal)
{
  bool exists = start->get_tangential_double_circle(*circle2, *circle3, *goal, m_ignore_obstacles);

  if(!exists) {
    return NOT_FREE;
  }

  float w = start->weight(m_ignore_obstacles)
      + circle2->weight(m_ignore_obstacles)
      + circle3->weight(m_ignore_obstacles)
      + goal->weight(m_ignore_obstacles);

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
  if (m_min_length<0.001) {
    std::cout << "WARNING:curve with cost 0"<<std::endl;
    return false;
  }  else {
    return m_min_length < NOT_FREE;
  }
}


double Curve::weight()
{
  double weight=0.0;
  for (unsigned i=0;i<m_min_combo.size();++i) {
    weight+=m_min_combo[i]->weight(m_ignore_obstacles);
  }
  if (fabs(weight-m_min_length)>1) {
    //std::cout << "weight:"<<weight<< " minlength"<<m_min_length<< std::endl;
    //std::cout.flush();
  }
  return weight;
}

void Curve::reset_iteration()
{
  if(m_min_combo.size() > 0){
    m_min_combo[0]->reset_iteration();
    m_output_number = 0;
  }

  m_iterating = true;
}

bool Curve::has_next()
{
  if(!m_iterating){
    std::cerr << "*** [RS] SEVERE CODE ERROR, ITERATION NOT STARTET ***" << std::endl;
  }
  if( m_min_length < NOT_FREE &&
      m_output_number < m_min_combo.size() ) {

    return m_min_combo[m_output_number]->has_next() || m_output_number < m_min_combo.size() - 1;

  } else {
    m_iterating = false;
    return false;
  }
}

Pose2d Curve::next()
{
  if(!m_iterating){
    std::cerr << "*** [RS] SEVERE CODE ERROR, ITERATION NOT STARTET ***" << std::endl;
  }

  CurveSegment * current_segment = m_min_combo[m_output_number];

  if(current_segment->has_next()) {
    // current segment has another point
    return current_segment->next();

  } else if(has_next()) {
    // current segment doesn't have another point, but there's another segment
    m_output_number++;
    m_min_combo[m_output_number]->reset_iteration();
    return next();

  } else {
    // there are no more points, should never happen if has_next() is used.
    std::cerr << "*** [RS] SEVERE CODE ERROR, DID NOT CALL HAS_NEXT ***" << std::endl;
    return Pose2d();
  }
}
