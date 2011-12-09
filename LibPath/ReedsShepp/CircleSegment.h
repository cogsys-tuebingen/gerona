/*
 * CircleSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CIRCLESEGMENT_H
#define CIRCLESEGMENT_H

#include "CurveSegment.h"
#include "LineSegment.h"

#include "../common/Point2d.h"

namespace ReedsShepp {

class CircleSegment: public CurveSegment
{
public:
  enum ORIENTATION {
    LEFT,
    RIGHT
  };

  enum MODE {
    AWAY_FROM_POINT,
    TOWARDS_POINT
  };

public:
  /**
   * Constructor, that sets the orientation and the direction
   *
   * @param orientation LEFT or RIGHT
   * @param direction FORWARD or BACKWARD
   */
  CircleSegment(ORIENTATION orientation, DIRECTION direction);

  /**
   * Computes a circle, that touches this circle and circle3 both tangentially.
   * This computation takes into consideration the orientation and direction of all three circles.
   * Since there are two possibilities, the combination with the lower weight is chosen.
   *
   * @param circle2 gets the information computed written into
   * @param circle3 the other circle, that must be touched
   */
  bool get_tangential_circle(CircleSegment &circle2, CircleSegment &circle3);

  /**
   * Computes two touching circles, that touch this circle and circle4 both tangentially.
   * This computation takes into consideration the orientation and direction of all four circles.
   * Since there are two possibilities, the combination with the lower weight is chosen.
   *
   * @param circle2 gets the information computed written into
   * @param circle3 gets the information computed written into
   * @param circle4 the other circle, that must be touched
   */
  bool get_tangential_double_circle(CircleSegment &circle2, CircleSegment &circle3, CircleSegment &circle4);


  /**
   * Computes a tangent, that touches this circle and the given one.
   * This computation takes into consideration the orientation and direction of both circles and the tangent.
   * The tangent fits onto both circles such that a continous trajectory (respecting the 3 directions) results.
   *
   * @param circle the second circle for the tangent
   * @param line_is_reverse
   */
  bool get_common_tangent(CircleSegment &circle, LineSegment &out_tangent, bool line_is_reverse);

  /**
   * Sets the center for this circle
   *
   * @param center point in map coordinates
   */
  void set_center_of_rotation(Point2d &center);

  /**
   * Sets the radius of the circle
   *
   * @param radius radius of the circle, NOT maximum steering angle!
   */
  void set_curve_radius(double radius);

  /**
   * Sets the orientation of the circle
   *
   * @param angle rotation of the "null-point"
   */
  void set_null_direction_angle(double angle);

  void set_end_angle(double angle);

  /**
   * Sets the role of this circle
   *
   * @param mode TOWARDS_POINT, if the circle is the goal, or
   *             AWAY_FROM_POINT, if the circle is the start
   */
  void set_mode(MODE mode);

  /**
   * Getter for the orientation
   */
  ORIENTATION orientation();

  /**
   * Getter for the center
   */
  Point2d center_of_orientation();

  /**
   * Getter for the curve radius
   */
  double radius();

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
   * Computes the weight of this segment
   */
  virtual float weight();

private:
  bool get_tangential_circle_helper(CircleSegment &circle2, CircleSegment &circle3, bool choose_positive_solution);
  bool get_tangential_double_circle_helper(CircleSegment &circle2, CircleSegment &circle3, CircleSegment &circle4, bool choose_positive_solution);

  void compute_arc_span(bool is_center_circle);

  ORIENTATION m_orientation;
  MODE m_mode;

  Point2d m_center;
  double m_radius;

  double m_angle_start;
  double m_tangent_angle;

  double m_arc_span;
  double m_stepsize;

  bool m_iterating;
  int m_output;
  int m_steps;
  int m_jump_over;
};

}

#endif // CIRCLESEGMENT_H
