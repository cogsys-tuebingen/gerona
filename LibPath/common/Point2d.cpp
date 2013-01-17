/*
 * Point2d.cpp
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "Point2d.h"

using namespace lib_path;

Point2d::Point2d()
{
}

Point2d::Point2d(double px, double py)
{
    this->x = px;
    this->y = py;
}

Point2d::Point2d(const Pose2d& p)
{
    x = p.x;
    y = p.y;
}

Point2d Point2d::from_polar(double angle, double radius)
{
    return Point2d(std::cos(angle) * radius, std::sin(angle) * radius);
}

Point2d Point2d::rotate(double angle) const
{
    Point2d p;
    p.x = cos(angle) * x - sin(angle) * y;
    p.y = sin(angle) * x + cos(angle) * y;
    return p;
}

Point2d Point2d::ortho() const
{
    Point2d p;
    p.x = y;
    p.y = -x;
    return p;
}

Point2d Point2d::normalize() const
{
    return resize(1);
}

Point2d Point2d::resize(double size) const
{
    double f = size / length();

    Point2d p;
    p.x = x * f;
    p.y = y * f;
    return p;
}

double Point2d::distance_to_origin() const
{
    return sqrt(pow(x, 2) + pow(y, 2));
}

double Point2d::length() const
{
    return distance_to_origin();
}

double Point2d::angle() const
{
    return atan2(y, x);
}
