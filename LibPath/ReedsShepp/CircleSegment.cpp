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
    : CurveSegment(direction), m_orientation(orientation),
      m_angle_start(0), m_angle_end(0), m_arc_length(0),
      m_iterating(false), m_output(0)
{
}

CurveSegment* CircleSegment::clone() const
{
    return new CircleSegment(*this);
}

CircleSegment::ORIENTATION CircleSegment::orientation()
{
    return m_orientation;
}

bool CircleSegment::has_orientation(ORIENTATION orientation)
{
    return m_orientation == orientation;
}

void CircleSegment::set_center_of_rotation(Point2d& center)
{
    m_center = center;
}

Point2d CircleSegment::get_center() const
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

void CircleSegment::set_start_angle_for_orientation(double orientation)
{
    if(m_orientation == CircleSegment::LEFT_CURVE) {
        m_angle_start = orientation - DTOR(90);
    } else {
        m_angle_start = orientation + DTOR(90);
    }

    m_angle_start = NORMALIZE(m_angle_start);

    compute_arc_length();
}

void CircleSegment::set_end_angle_for_orientation(double orientation)
{
    if(m_orientation == CircleSegment::LEFT_CURVE) {
        m_angle_end = orientation - DTOR(90);
    } else {
        m_angle_end = orientation + DTOR(90);
    }

    m_angle_end = NORMALIZE(m_angle_end);

    compute_arc_length();
}

void CircleSegment::set_start_angle(double angle)
{
    m_angle_start = NORMALIZE(angle);

    compute_arc_length();
}

void CircleSegment::set_end_angle(double angle)
{
    m_angle_end = NORMALIZE(angle);

    compute_arc_length();
}

double CircleSegment::get_start_angle() const
{
    return m_angle_start;
}

double CircleSegment::get_end_angle() const
{
    return m_angle_end;
}

Point2d CircleSegment::get_start_point() const
{
    return m_center + Point2d::from_polar(m_angle_start, m_radius);
}

Point2d CircleSegment::get_end_point() const
{
    return m_center + Point2d::from_polar(m_angle_end, m_radius);
}

float CircleSegment::weight(bool ignore_obstacles)
{
    m_weight = 0;
    bool is_free = checkCircle(m_center.x, m_center.y, m_radius, ignore_obstacles);

    if(ignore_obstacles || is_free) {
        float cost = (m_direction == CurveSegment::BACKWARD) ? m_cost_backwards : m_cost_forwards;
        if(m_use_map_cost)
            return cost * m_weight * m_cost_curve / m_map->getResolution();
        else
            return fabs(m_arc_length) * m_radius * cost * m_cost_curve;
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
    while(x >= y) {
        is_free &= test8Points(cx, cy, x, y, ignore_obstacles);

        error += y;
        ++y;
        error += y;

        // The following test may be implemented in assembly language in
        // most machines by testing the carry flag after adding 'y' to
        // the value of 'error' in the previous step, since 'error'
        // nominally has a negative value.
        if(error >= 0) {
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
    if(x != y) {
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
    if(x != 0) is_free &= testPixel(cx, cy, cx - x, cy + y, ignore_obstacles);
    if(y != 0) is_free &= testPixel(cx, cy, cx + x, cy - y, ignore_obstacles);
    if(x != 0 && y != 0) is_free &= testPixel(cx, cy, cx - x, cy - y, ignore_obstacles);

    return is_free;
}

bool CircleSegment::testPixel(int cx, int cy, int x, int y, bool ignore_obstacles)
{
    if(m_map->isInMap(x, y)) {
        float angle = atan2(y-cy, x-cx);
        bool on_segment = false;

        float start = NORMALIZE(m_angle_start);
        float end = NORMALIZE(m_angle_end);

        bool increasing = m_angle_step_size > 0;//(m_arc_length > 0) ^ (m_direction == BACKWARD);
        bool decreasing = !increasing;

        if(start > end) {
            if(decreasing) {
                // start > end  &&  angle decreasing  -> no possible overflow
                on_segment = angle <= start && angle >= end;
            } else {
                // start > end  &&  angle increasing  ->  possible overflow
                on_segment = angle >= start || angle <= end;
            }
        } else {
            // start <= end
            if(increasing) {
                // start <= end  &&  angle increasing  -> no possible overflow
                on_segment = angle >= start && angle <= end;
            } else {
                // start <= end  &&  angle decreasing  -> possible overflow
                on_segment = angle <= start || angle >= end;
            }
        }

        if(on_segment) {
            m_weight += max(m_min_cell_cost, m_map->getValue(x, y));
            bool free = m_map->isFree(x, y);
            if(m_trace != -1) {
                m_map->setValue(x, y, free ? 240 : 40);
            }
            if(ignore_obstacles) {
                return true;
            } else {
                return free;
            }
        }
    }

    return true;
}

void CircleSegment::compute_arc_length()
{
    float arc_angle = NORMALIZE(m_angle_end - m_angle_start);

    // arc length is in [-2pi, 2pi] and represents the arc to drive on
    //  (may be up to a full circle in both directions)

    // arc length is positive if (mathematical definition of a positive radian):
    //   - left curve and driving forward
    //   - right curve and driving backward
    // arc length is negative if:
    //   - left curve and driving backward
    //   - right curve and driving forward
    //
    //      Orientation  Direction    Sign
    //       LEFT         FORWARD      +
    //       LEFT         BACKWARD     -
    //       RIGHT        FORWARD      -
    //       RIGHT        BACKWARD     +
    //
    //   => POSITIVE = (LEFT xor BACKWARD)

    bool must_be_positive = (m_orientation == LEFT_CURVE) ^ (m_direction == BACKWARD);
    bool is_positive = (arc_angle >= 0);

    // if arc must be positive, but is negative
    //  -> arc has to be > PI and has been normalized to a negative number -> add 2 PI
    // similarly, if arc must be negative, but is positive
    //  -> subtract 2 PI
    if(must_be_positive && !is_positive){
        m_arc_length = arc_angle + 2 * M_PI;

    } else if(!must_be_positive && is_positive){
        m_arc_length = arc_angle - 2 * M_PI;

    } else {
        m_arc_length = arc_angle;
    }

    int sign = m_arc_length >= 0 ? 1 : -1;

    // each step should produce a distance of m_max_distance
    // -> polar coordinates, calulate step in angle
    m_angle_step_size = m_max_distance / m_radius * sign;

    // no of steps
    m_steps = std::max(1, (int) std::abs(std::floor(m_arc_length / m_angle_step_size)));
}

void CircleSegment::reset_iteration()
{
    m_iterating = true;
    m_output = 0;

    compute_arc_length();
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

    float angle = m_angle_start + m_angle_step_size * m_output;
    Point2d pt = m_center + ray.rotate(angle);

    Pose2d ret;
    ret.x = pt.x;
    ret.y = pt.y;
    ret.theta = angle + ((m_orientation == RIGHT_CURVE) ? DTOR(-90) : DTOR(90));

    m_output++;

    return ret;
}
