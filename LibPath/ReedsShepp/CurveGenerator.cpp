/*
 * CurveGenerator.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "CurveGenerator.h"

/// SYSTEM
#include <stdio.h>
#include <typeinfo>

using namespace lib_path;

CurveGenerator::CurveGenerator()
    : m_use_map_cost(false), m_min_cell_cost(10),
      m_circle_radius(2.0), m_max_waypoint_distance(1.0),
      m_cost_forwards(1.0), m_cost_backwards(1.0), m_cost_curve(1.0), m_cost_straight(1.0),
      m_trace(-1)
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

void CurveGenerator::set_trace(int value)
{
    m_trace = value;
}

void CurveGenerator::set_use_map_cost(bool arg)
{
    m_use_map_cost = arg;
}

void CurveGenerator::set_min_cell_cost(uint8_t cost)
{
    m_min_cell_cost = cost;
}

bool CurveGenerator::parse(const std::string& sequence, std::ostream& out)
{
    CurveSegment::DIRECTION direction = CurveSegment::FORWARD;
    int chars = 0;
    bool error = false;

    bool is_sub_pattern = false;

    std::string::const_iterator it;

    Curve new_curve;

    for(it=sequence.begin(); it!=sequence.end() && !error; ++it, ++chars) {
        // SUBPATTERN
        if(is_sub_pattern) {
            switch((*it)) {
            case '(':
                out << "RS Generator: error parsing" << std::endl << sequence << ", only one nesting level allowed" << std::endl;
                error = true;
                break;
            case 'b': {
                const CurveSegment& cs = new_curve.get_segment(new_curve.count() - 1);
//                CurveSegment* cs = m_sequences[m_count].at(m_sequences[m_count].size()-1);

                if(typeid(cs) == typeid(CircleSegment)) {
                    //CircleSegment * circle = dynamic_cast<CircleSegment*> (cs);
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
                new_curve.add(LineSegment(direction));
                break;
            case 'R':
                new_curve.add(CircleSegment(CircleSegment::RIGHT_CURVE, direction));
                break;
            case 'L':
                new_curve.add(CircleSegment(CircleSegment::LEFT_CURVE, direction));
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

    if(!error) {

        if(is_sub_pattern) {
            out << "RS Generator: error parsing" << std::endl << sequence << ", nesting not closed" << std::endl;
            error = true;
        } else {
            m_curves.push_back(new_curve);
        }
    } else {
        for(int i=0; i < chars; ++i)
            out << " ";
        out << "^" << std::endl;
    }
    return error;
}

Curve* CurveGenerator::find_path(const Pose2d& start, const Pose2d& goal, GridMap2d* map, bool ignore_obstacles)
{
    Curve* best_curve = NULL;
    double best_weight = INFINITY;

    for(std::vector<Curve>::iterator c = m_curves.begin(); c != m_curves.end(); ++c){
        c->m_start = start;
        c->m_goal = goal;
        c->m_map = map;

        c->m_ignore_obstacles = ignore_obstacles;

        c->m_circle_radius = m_circle_radius / map->getResolution();
        c->m_max_waypoint_distance = m_max_waypoint_distance / map->getResolution();

        c->m_use_map_cost = m_use_map_cost;
        c->m_min_cell_cost = m_min_cell_cost;
        c->m_cost_backwards = m_cost_backwards;
        c->m_cost_forwards = m_cost_forwards;
        c->m_cost_curve = m_cost_curve;
        c->m_cost_straight = m_cost_straight;

        c->m_trace = m_trace;

        double weight = c->check_if_admissible();

        if(weight < best_weight){
            best_curve = &*c;
            best_weight = weight;
        }
    }

    return new Curve(*best_curve);
}
