/*
 * Curve.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "Curve.h"

/// PROJECT
#include "../common/MapMath.h"

/// COMPONENT
#include <iostream>
#include <typeinfo>

using namespace lib_path;

Curve::Curve()
    : m_ignore_obstacles(false),
      m_circle_radius(10.0), m_max_waypoint_distance(10.0),
      m_use_map_cost(false), m_min_cell_cost(10),
      m_cost_forwards(1.0), m_cost_backwards(1.0), m_cost_curve(1.0), m_cost_straight(1.0),
      m_weight(NOT_FREE),
      m_iterating(false), m_output_number(0), m_trace(-1)
{
}

Curve::Curve(const Curve& c)
    : m_ignore_obstacles(c.m_ignore_obstacles),
      m_circle_radius(c.m_circle_radius), m_max_waypoint_distance(c.m_max_waypoint_distance),
      m_use_map_cost(c.m_use_map_cost), m_min_cell_cost(c.m_min_cell_cost),
      m_cost_forwards(c.m_cost_forwards), m_cost_backwards(c.m_cost_backwards),
      m_cost_curve(c.m_cost_curve), m_cost_straight(c.m_cost_straight),
      m_weight(c.m_weight),
      m_circle_start(c.m_circle_start), m_circle_goal(c.m_circle_goal),
      m_start(c.m_start), m_goal(c.m_goal),
      m_map(c.m_map),
      m_iterating(c.m_iterating), m_output_number(c.m_output_number),
      m_trace(c.m_trace)
{
    for(std::vector<CurveSegment*>::const_iterator it = c.m_sequence.begin();
            it != c.m_sequence.end(); ++it) {
        m_sequence.push_back((*it)->clone());
    }
}

Curve::~Curve()
{
    for(std::vector<CurveSegment*>::const_iterator it = m_sequence.begin();
            it != m_sequence.end(); ++it) {
        delete *it;
    }
}

Pose2d Curve::start()
{
    return m_start;
}

Pose2d Curve::goal()
{
    return m_goal;
}

void Curve::set_trace(int value)
{
    m_trace = value;
}

int Curve::count()
{
    return m_sequence.size();
}

const CurveSegment& Curve::get_segment(int index)
{
    assert(index >= 0);
    assert(index < (int) m_sequence.size());

    return *m_sequence[index];
}

void Curve::add(const CurveSegment& segment)
{
    CurveSegment* copy = segment.clone();

    assert(copy != NULL);

    m_sequence.push_back(copy);
}

double Curve::check_if_admissible()
{
    m_weight = NOT_FREE;

    if(m_sequence.size() < 3 || m_sequence.size() > 4) {
        std::cerr << "invalid sequence" << std::endl;
        return NOT_FREE;
    }

    if(m_start.distance_to(m_goal) < 2) {
        // TODO: robot is 2 pixels away from the target, what to do?
        return NOT_FREE;
    }

    if(m_sequence.size() < 3) {
        std::cerr << "invalid sequence, length " << m_sequence.size() << " < 3" << std::endl;
        return NOT_FREE;
    }

    init_segments();

    if(!handle_sequence()) {
        std::cerr << "unknown sequence" << std::endl;
    }

    return m_weight;
}

void Curve::init_segments()
{
    init_circle_pairs(m_start, m_circle_start);
    init_circle_pairs(m_goal, m_circle_goal);

    std::vector<CurveSegment*>::iterator it;

    for(it=m_sequence.begin(); it!=m_sequence.end(); ++it) {
        CurveSegment* c = *it;

        assert(c != NULL);

        c->set_map(m_map);

        c->set_max_distance(m_max_waypoint_distance);

        c->set_use_map_cost(m_use_map_cost);
        c->set_min_cell_cost(m_min_cell_cost);
        c->set_cost_backwards(m_cost_backwards);
        c->set_cost_forwards(m_cost_forwards);
        c->set_cost_curve(m_cost_curve);
        c->set_cost_straight(m_cost_straight);

        c->set_trace(m_trace);

        if(typeid(**it) == typeid(CircleSegment)) {
            CircleSegment* c = dynamic_cast<CircleSegment*>(*it);
            c->set_radius(m_circle_radius);
        }
    }

    CircleSegment*  start = dynamic_cast<CircleSegment*>(m_sequence[0]);
    CircleSegment*   goal = dynamic_cast<CircleSegment*>(m_sequence[m_sequence.size() - 1]);

    assert(start && goal);

    if(start->get_orientation() == CircleSegment::LEFT_CURVE)
        start->set_center(m_circle_start.center_left);
    else
        start->set_center(m_circle_start.center_right);

    if(goal->get_orientation() == CircleSegment::LEFT_CURVE)
        goal->set_center(m_circle_goal.center_left);
    else
        goal->set_center(m_circle_goal.center_right);

    start->set_start_angle_for_orientation(m_start.theta);
    goal->set_end_angle_for_orientation(m_goal.theta);
}

bool Curve::handle_sequence()
{
    if(m_sequence.size() == 3) {
        if(typeid(*m_sequence[1]) == typeid(CircleSegment)) {
            m_weight = handle_sequence_C3();
            return true;

        } else {
            m_weight = handle_sequence_CSC();
            return true;
        }

    } else if(m_sequence.size() == 4) {
        if(typeid(*m_sequence[1]) == typeid(CircleSegment) && typeid(*m_sequence[2]) == typeid(CircleSegment)) {
            m_weight = handle_sequence_C4();
            return true;
        }
    }

    return false;
}

double Curve::handle_sequence_CSC()
{
    CircleSegment* circle1 = dynamic_cast<CircleSegment*>(m_sequence[0]);
    LineSegment* line = dynamic_cast<LineSegment*>(m_sequence[1]);
    CircleSegment* circle2 = dynamic_cast<CircleSegment*>(m_sequence[2]);

    assert(circle1 && circle2 && line);

    if(!GeometryHelper::get_common_tangent(*circle1, *circle2, *line)) {
        return NOT_FREE;
    }

    float w = circle1->weight(m_ignore_obstacles)
              + line->weight(m_ignore_obstacles)
              + circle2->weight(m_ignore_obstacles);
    return w;
}

double Curve::handle_sequence_C3()
{
    CircleSegment* circle1 = dynamic_cast<CircleSegment*>(m_sequence[0]);
    CircleSegment* circle2 = dynamic_cast<CircleSegment*>(m_sequence[1]);
    CircleSegment* circle3 = dynamic_cast<CircleSegment*>(m_sequence[2]);

    assert(circle1 && circle2 && circle3);

    if(!GeometryHelper::get_tangential_circle(*circle1, *circle2, *circle3, m_ignore_obstacles)) {
        return NOT_FREE;
    }

    float w = circle1->weight(m_ignore_obstacles)
              + circle2->weight(m_ignore_obstacles)
              + circle3->weight(m_ignore_obstacles);
    return w;
}

double Curve::handle_sequence_C4()
{
    CircleSegment* start = dynamic_cast<CircleSegment*>(m_sequence[0]);
    CircleSegment* circle2 = dynamic_cast<CircleSegment*>(m_sequence[1]);
    CircleSegment* circle3 = dynamic_cast<CircleSegment*>(m_sequence[2]);
    CircleSegment* goal = dynamic_cast<CircleSegment*>(m_sequence[3]);

    assert(start && circle2 && circle3 && goal);

    if(!GeometryHelper::get_tangential_double_circle(*start, *circle2, *circle3, *goal, m_ignore_obstacles)) {
        return NOT_FREE;
    }

    float w = start->weight(m_ignore_obstacles);
    w += circle2->weight(m_ignore_obstacles);
    w += circle3->weight(m_ignore_obstacles);
    w += goal->weight(m_ignore_obstacles);

    return w;
}

void Curve::init_circle_pairs(Pose2d& next_to, circle_pair& target)
{
    Point2d dir = Point2d::from_polar(next_to.theta + M_PI / 2, m_circle_radius);

    target.center_left = Point2d(next_to.x + dir.x, next_to.y + dir.y);
    target.center_right = Point2d(next_to.x - dir.x, next_to.y - dir.y);
}


bool Curve::is_valid()
{
    if(m_weight < 0.001) {
        std::cout << "WARNING:curve with cost 0"<<std::endl;
        return false;
    }  else {
        return m_weight < NOT_FREE;
    }
}
double Curve::weight() const
{
    return m_weight;
}

void Curve::reset_iteration()
{
    for(std::vector<CurveSegment*>::const_iterator it = m_sequence.begin();
            it != m_sequence.end(); ++it) {

        (*it)->reset_iteration();
        m_output_number = 0;
    }

    m_iterating = true;
}

bool Curve::has_next()
{
    if(!m_iterating) {
        std::cerr << "*** [RS] SEVERE CODE ERROR, ITERATION NOT STARTET ***" << std::endl;
    }

    if(m_output_number < m_sequence.size()) {

        return m_sequence[m_output_number]->has_next() || m_output_number < m_sequence.size() - 1;

    } else {
        m_iterating = false;
        return false;
    }
}

Pose2d Curve::next()
{
    if(!m_iterating) {
        std::cerr << "*** [RS] SEVERE CODE ERROR, ITERATION NOT STARTET ***" << std::endl;
    }

    CurveSegment* current_segment = m_sequence[m_output_number];

    if(current_segment->has_next()) {
        // current segment has another point
        return current_segment->next();

    } else if(has_next()) {
        // current segment doesn't have another point, but there's another segment
        m_output_number++;
        m_sequence[m_output_number]->reset_iteration();
        return next();

    } else {
        // there are no more points, should never happen if has_next() is used.
        std::cerr << "*** [RS] SEVERE CODE ERROR, DID NOT CALL HAS_NEXT ***" << std::endl;
        return Pose2d();
    }
}
