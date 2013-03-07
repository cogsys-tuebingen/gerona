/*
 * GeometryHelper.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "GeometryHelper.h"

using namespace lib_path;

GeometryHelper::GeometryHelper()
{
}

bool GeometryHelper::get_common_tangent(CircleSegment& circle_from, CircleSegment& circle_to, LineSegment& tangent_out)
{
    double distance = (circle_from.get_center() - circle_to.get_center()).length();
    double radius_sum = circle_from.m_radius + circle_to.m_radius;

    // the circles overlap, if the distance between their centers is <= the sum of their radii
    // in this case, we cannot compute a tangent
    if(distance < radius_sum) {
        return false;
    }

    if(circle_from.m_orientation == circle_to.m_orientation) {
        external_tangent(circle_from, tangent_out, circle_to);
    } else {
        internal_tangent(circle_from, tangent_out, circle_to);
    }

    return true;
}

void GeometryHelper::external_tangent(CircleSegment& circle_from, LineSegment& tangent_out, CircleSegment& circle_to)
{
    /*  external tangent
     *    p         q
     *   _|_ __>__ _|_
     *  /   \     /   \
     *  | s |     | g |
     *  \___/     \___/
     *
     **/

    const Point2d s = circle_from.m_center;
    const Point2d g = circle_to.m_center;

    Point2d direction = g - s;
    float tangent_angle = direction.angle();

    float tangent_orientation = tangent_angle;

    bool line_is_reverse = tangent_out.direction() == CurveSegment::BACKWARD;
    if(line_is_reverse) {
        tangent_orientation -= DTOR(180);
    }

    circle_from.set_end_angle_for_orientation(tangent_orientation);
    circle_to.set_start_angle_for_orientation(tangent_orientation);

    Point2d offset = direction.ortho().normalize() * circle_from.m_radius;

    if((circle_from.m_orientation == CircleSegment::RIGHT_CURVE) ^ line_is_reverse)
        offset = offset * -1;

    Point2d p = s + offset;
    Point2d q = g + offset;

    tangent_out.set_points(p, q);
}

void GeometryHelper::internal_tangent(CircleSegment& circle_from, LineSegment& tangent_out, CircleSegment& circle_to)
{
    /*  internal tangent
     *      p
     *   ___|    ___
     *  /   \\  /   \
     *  | s | \ | g |
     *  \___/  \\___/
     *          |
     *          q
     **/

    const Point2d s = circle_from.m_center;
    const Point2d g = circle_to.m_center;

    Point2d direction = g - s;


    // gamma is the angle between the vectors (p - s) and (g - s)
    float gamma = acos((circle_from.m_radius + circle_to.m_radius) / direction.length());

    float orientation = direction.angle();

    // since gamma is computed via arccos, it is always positive
    // -> there are two inner tangents (+- gamma)
    //                 SIGN
    //  LEFT  FORWARD   -
    //  RIGHT FORWARD   +
    //  LEFT  BACKWARD  -
    //  RIGHT BACKWARD  +

    bool use_positive_gamma = (circle_from.m_orientation == CircleSegment::RIGHT_CURVE);
    if(!use_positive_gamma) {
        gamma = -gamma;
    }

    // if the tangent is driven in reverse, we need to take the other tangent
    bool line_is_reverse = tangent_out.direction() == CurveSegment::BACKWARD;
    if(line_is_reverse) {
        gamma = -gamma;
    }

    circle_from.set_end_angle(orientation + gamma);
    circle_to.set_start_angle(orientation + gamma + M_PI);

    Point2d offset = circle_from.get_end_point() - circle_from.get_center();

    Point2d p = s + offset;
    Point2d q = g - offset;

    tangent_out.set_points(p, q);
}

bool GeometryHelper::get_tangential_circle(CircleSegment& circle1, CircleSegment& circle2, CircleSegment& circle3,
        bool ignore_obstacles)
{
    std::vector<CircleSegment*> segments;
    segments.push_back(&circle1);
    segments.push_back(&circle2);
    segments.push_back(&circle3);

    return symmetry_helper(segments, 4, ignore_obstacles, boost::bind(&GeometryHelper::calculate_tangential_circle_vector, _1, _2));
}

bool GeometryHelper::get_tangential_double_circle(CircleSegment& circle_from, CircleSegment& circle2, CircleSegment& circle3, CircleSegment& circle_to,
        bool ignore_obstacles)
{
    std::vector<CircleSegment*> segments;
    segments.push_back(&circle_from);
    segments.push_back(&circle2);
    segments.push_back(&circle3);
    segments.push_back(&circle_to);

    return symmetry_helper(segments, 6, ignore_obstacles, boost::bind(&GeometryHelper::test_tangential_double_circle_vector, _1, _2));
}

bool GeometryHelper::calculate_tangential_circle_vector(const std::vector<CircleSegment*>& segments, bool choose_positive_solution)
{
    assert(segments.size() == 3);

    return calculate_tangential_circle(*segments[0], *segments[1], *segments[2], choose_positive_solution);
}


bool GeometryHelper::calculate_tangential_circle(CircleSegment& circle_from, CircleSegment& circle2, CircleSegment& circle_to,
        bool choose_positive_solution)
{
    /*      |----------------| delta
     *      |-------|  delta / 2
     *    ______    |     ______
     *   /      \   |    /      \
     *  /        \  |   /        \
     *  |   s    |  |   |   g    |
     *  |        |__|___|        |
     *  \        /  |   \        /
     *   \______/   |    \______/
     *          |   C    |
     *          |        |
     *          \        /
     *           \______/
     *
     *  gamma = <) ( (g - s), (C - s) )
     *
     *               /  delta  \   /
     *  cos(gamma) = |  -----  |  /  2 r
     *               \    2    / /
     *
     */

    const Point2d& s = circle_from.m_center;
    const Point2d& g = circle_to.m_center;

    Point2d delta = (g - s);

    double r = circle_from.m_radius;
    double diameter = 2 * r;

    double theta = delta.angle();
    double gamma = acos((delta.length() / 2) / diameter);
    if(!choose_positive_solution) {
        gamma = -gamma;
    }

    Point2d center_1_to_2 = Point2d::from_polar(gamma + theta, diameter);

    Point2d C = s + center_1_to_2;

    circle2.set_center(C);

    circle_from.set_end_angle((C - s).angle());
    circle_to.set_start_angle((C - g).angle());

    circle2.set_start_angle(circle_from.get_end_angle() + M_PI);
    circle2.set_end_angle(circle_to.get_start_angle() + M_PI);

    return true;
}

bool GeometryHelper::test_tangential_double_circle_vector(const std::vector<CircleSegment*>& segments, bool choose_positive_solution)
{
    assert(segments.size() == 4);

    return test_tangential_double_circle(*segments[0], *segments[1], *segments[2], *segments[3], choose_positive_solution);
}

bool GeometryHelper::test_tangential_double_circle(CircleSegment& circle_from, CircleSegment& circle2, CircleSegment& circle3, CircleSegment& circle_to,
        bool choose_positive_solution)
{
    /*      |-------------------------| delta
     *      |-------------| delta / 2
     *    r |----|
     *           |--------| delta / 2 - r
     *    ______          |         ______
     *   /      \         |        /      \
     *  /        \        |       /        \
     *  |   s    |        |       |   g    |
     *  |        |______  | ______|        |
     *  \        /      \  /      \        /
     *   \______/        \/        \______/
     *          |   C2   ||   C3   |
     *          |        ||        |
     *          \        /\        /
     *           \______/  \______/

     *
     *  gamma = <) ( (g - s), (C2 - s) )
     *
     *               /  delta     \   /
     *  cos(gamma) = |  ----- - r |  /  2 r
     *               \    2       / /
     *
     */

    const Point2d& s = circle_from.get_center();
    const Point2d& g = circle_to.get_center();

    Point2d delta = (g - s);

    double r = circle_from.m_radius;
    double diameter = 2 * r;

    double theta = delta.angle();
    double gamma = acos((delta.length() / 2.0 - r) / diameter);
    if(!choose_positive_solution) {
        gamma = -gamma;
    }

    Point2d center_1_to_2 = Point2d::from_polar(gamma + theta, diameter);
    Point2d center_2_to_3 = Point2d::from_polar(theta, diameter);

    Point2d C2 = s + center_1_to_2;
    Point2d C3 = C2 + center_2_to_3;

    circle2.set_center(C2);
    circle3.set_center(C3);

    circle2.set_end_angle(center_2_to_3.angle());
    circle3.set_start_angle(center_2_to_3.angle() + M_PI);

    circle_from.set_end_angle((C2 - s).angle());
    circle_to.set_start_angle((C3 - g).angle());

    circle2.set_start_angle(circle_from.get_end_angle() + M_PI);
    circle3.set_end_angle(circle_to.get_start_angle() + M_PI);

    return true;
}


bool GeometryHelper::symmetry_helper(const std::vector<CircleSegment*>& circles, double max_dist, bool ignore_obstacles,
                                     boost::function<bool (const std::vector<CircleSegment*>&, bool)> callback)
{
    const CircleSegment& circle_from = *circles.at(0);
    const CircleSegment& circle_to = *circles.at(circles.size() - 1);

    const Point2d& front = circle_from.get_center();
    const Point2d& back = circle_to.get_center();

    if((front - back).length() < 2) {
        // c1 and c4 only one pixel apart
        return false;
    }

    if((front - back).length() > max_dist * circle_from.m_radius) {
        // c1 and c4 too far apart
        return false;
    }

    float weight_positive = NOT_FREE;
    float weight_negative = NOT_FREE;

    // here we have to symmetric cases, test them both an use the better one
    if(callback(circles, true)) {
        weight_positive = 0;
        for(std::vector<CircleSegment*>::const_iterator it = circles.begin(); it != circles.end(); ++it) {
            weight_positive += (*it)->weight(ignore_obstacles);
        }
    }
    if(callback(circles, false)) {
        weight_negative = 0;
        for(std::vector<CircleSegment*>::const_iterator it = circles.begin(); it != circles.end(); ++it) {
            weight_negative += (*it)->weight(ignore_obstacles);
        }
    }

    if(weight_negative >= NOT_FREE && weight_positive >= NOT_FREE) {
        return false;

    } else {
        if(weight_positive < weight_negative) {
            callback(circles, true);
        } else {
            callback(circles, false);
        }
        return true;
    }
}

