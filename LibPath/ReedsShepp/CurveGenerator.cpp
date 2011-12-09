/*
 * CurveGenerator.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "CurveGenerator.h"

#include <iostream>
#include <limits>
#include <vector>
#include <typeinfo>

using namespace ReedsShepp;

CurveGenerator::CurveGenerator()
  : m_count(0), m_circle_radius(2.0), m_max_waypoint_distance(1.0),
    m_cost_forwards(1.0), m_cost_backwards(1.0), m_cost_curve(1.0), m_cost_straight(1.0)
{
}

void CurveGenerator::set_circle_radius(double circle_radius)
{
  m_circle_radius = circle_radius;
}

void CurveGenerator::set_max_waypoint_distance(double max_waypoint_distance)
{
  m_max_waypoint_distance = max_waypoint_distance;
}


void CurveGenerator::set_cost_forwards(double cost_forwards)
{
  m_cost_forwards = cost_forwards;
}

void CurveGenerator::set_cost_backwards(double cost_backwards)
{
  m_cost_backwards = cost_backwards;
}

void CurveGenerator::set_cost_curve(double cost_curve)
{
  m_cost_curve = cost_curve;
}

void CurveGenerator::set_cost_straight(double cost_straight)
{
  m_cost_straight = cost_straight;
}

bool CurveGenerator::parse(std::string sequence, std::ostream& out)
{
  CurveSegment::DIRECTION direction = CurveSegment::FORWARD;
  int chars = 0;
  bool error = false;

  bool is_sub_pattern = false;

  std::string::iterator it;

  for(it=sequence.begin(); it!=sequence.end() && !error; ++it, ++chars){
    // SUBPATTERN
    if(is_sub_pattern){
      switch((*it)) {
      case '(':
        out << "RS Generator: error parsing" << std::endl << sequence << ", only one nesting level allowed" << std::endl;
        error = true;
        break;
      case 'b':{
        CurveSegment * cs = m_sequences[m_count].at(m_sequences[m_count].size()-1);

        if(typeid(*cs) == typeid(CircleSegment)){
          CircleSegment * circle = dynamic_cast<CircleSegment*> (cs);
          //circle.set_part_of_turn();

        } else {
          out << "RS Generator: error parsing" << std::endl << sequence << ", only a circle can have a subsequence" << std::endl;
          error = true;
        }
      }
      break;
      case 'p':
        break;
      case ')':
        is_sub_pattern = false;
        break;
      case 'S':
      case 'R':
      case 'L':
      case '|':
        out << "RS Generator: error parsing" << std::endl << sequence << ", not allowed in subpattern" << std::endl;
        error = true;
        break;
      default:
        out << "RS Generator: error parsing" << std::endl << sequence << ", unknown char '" << (*it) << "'" << std::endl;
        error = true;
        break;
      }

      // NOT A SUBPATTERN
    } else {
      switch((*it)) {
      case '(':
        is_sub_pattern = true;
        break;
      case ')':
        out << "RS Generator: error parsing" << std::endl << sequence << ", no nesting to close here" << std::endl;
        error = true;
        break;
      case 'S':
        m_sequences[m_count].push_back(new LineSegment(direction));
        break;
      case 'R':
        m_sequences[m_count].push_back(new CircleSegment(CircleSegment::RIGHT, direction));
        break;
      case 'L':
        m_sequences[m_count].push_back(new CircleSegment(CircleSegment::LEFT, direction));
        break;
      case '|':
        if(direction == CurveSegment::FORWARD)
          direction = CurveSegment::BACKWARD;
        else
          direction = CurveSegment::FORWARD;
        break;
      default:
        out << "RS Generator: error parsing" << std::endl << sequence << ", unknown char '" << (*it) << "'" << std::endl;
        error = true;
        break;
      }
    }

    if(error)
      break;
  }

  if(!error){

    if(is_sub_pattern){
      out << "RS Generator: error parsing" << std::endl << sequence << ", nesting not closed" << std::endl;
      error = true;
    } else {
      m_count++;
    }
  } else {
    for(int i=0; i < chars; ++i)
      out << " ";
    out << "^" << std::endl;
  }
}

Curve * CurveGenerator::find_path(Pose2d &start, Pose2d &goal, MapInfo *map) {
  Curve * c = new Curve;

  c->m_start = start;
  c->m_goal = goal;
  c->m_map = map;

  c->m_circle_radius = m_circle_radius / map->resolution;
  c->m_max_waypoint_distance = m_max_waypoint_distance / map->resolution;

  c->m_min_length = std::numeric_limits<float>::max();

  c->m_cost_backwards = m_cost_backwards;
  c->m_cost_forwards = m_cost_forwards;
  c->m_cost_curve = m_cost_curve;
  c->m_cost_straight = m_cost_straight;



  // out << "map resolution: " << map->resolution << std::endl;
  // out << "max distance: " << m_max_waypoint_distance << " is " << c->m_max_waypoint_distance << " in map coordinates" << std::endl;

  for(int i=0; i<m_count; ++i) {
    c->test_sequence(m_sequences[i]);
  }

  return c;
}
