/*
 * LineSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef LINESEGMENT_H
#define LINESEGMENT_H

#include "CurveSegment.h"
#include "../common/Point2d.h"

namespace ReedsShepp {

class LineSegment: public CurveSegment
{
public:
  /**
   * Constructor, that sets the direction
   *
   * @param direction FORWARD or BACKWARD
   */
  LineSegment(DIRECTION direction);


  /**
   * Sets the points that describe the line
   *
   * @param start start point
   * @param end end point
   */
  void set_points(Point2d start, Point2d end);


  /**
   * Start iterating over the points on this segment
   */
  virtual void reset_iteration();

  /**
   * Check, if another point exists
   *
   * @return true, iff there exists another point
   */
  virtual bool has_next();

  /**
   * Get the next point in this iteration
   */
  virtual Pose2d next();



  /**
   * Get the start point
   */
  virtual Pose2d start();

  /**
   * Get the end point
   */
  virtual Pose2d end();



  /**
   * Computes the weight of this segment
   */
  virtual float weight();

private:
  Pose2d m_start;
  Pose2d m_end;

  bool m_iterating;
  int m_output;
  int m_segments;

  float m_slope;
};

}

#endif // LINESEGMENT_H
