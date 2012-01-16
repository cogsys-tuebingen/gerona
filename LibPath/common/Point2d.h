/*
 * Point2d.h
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef POINT2D_H
#define POINT2D_H

#include "Pose2d.h"
#include "MapMath.h"

namespace lib_path {

class Point2d
{
public:
  Point2d();
  Point2d(const Pose2d& p);
  Point2d(double x, double y);

  /**
   * Rotates the point around the origin
   *
   * @param angle angle in radians
   */
  Point2d rotate(double angle) const;

  /**
   * Compute a point so that the vectors through both points are orthogonal
   */
  Point2d ortho() const;

  /**
   * Computes the distance to the origin
   */
  double distance_to_origin() const;

  /**
   * Computes the angle between the x-axis and the line throug the point
   */
  double angle() const;


public:
  // x-coordinate
  double x;
  // y-coordinate
  double y;


  /*
   * Operators
   */
public:
  Point2d operator / (double f) const{
    Point2d p;
    p.x = x / f;
    p.y = y / f;
    return p;
  }

  Point2d operator * (double f) const{
    Point2d p;
    p.x = x * f;
    p.y = y * f;
    return p;
  }

  Point2d operator *= (double f){
    x *= f;
    y *= f;
    return *this;
  }

  Point2d operator + (const Point2d &rhs) const{
    Point2d p;
    p.x = x + rhs.x;
    p.y = y + rhs.y;
    return p;
  }

  Point2d operator - (const Point2d &rhs) const{
    Point2d p;
    p.x = x - rhs.x;
    p.y = y - rhs.y;
    return p;
  }
};

inline std::ostream& operator << (std::ostream& ostr, const Point2d& p){
  ostr << "(" << p.x << " / " << p.y << ")";
  return ostr;
}

} // namespace "lib_path"

#endif // POINT2D_H
