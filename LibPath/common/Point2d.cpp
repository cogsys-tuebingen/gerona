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

double Point2d::distance_to_origin() const
{
  return sqrt(pow(x, 2) + pow(y, 2));
}

double Point2d::angle() const
{
  return atan2(y, x);
}
