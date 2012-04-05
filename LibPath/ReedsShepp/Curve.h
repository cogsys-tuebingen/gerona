/*
 * Curve.h
 *
 *  Created on: Apr 2, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CURVE_H_
#define CURVE_H_

#include "CurveSegment.h"
#include "CircleSegment.h"
#include "LineSegment.h"

#include <vector>

namespace lib_path {

class circle_pair {
public:
  Point2d center_left;
  Point2d center_right;
};

class Curve {
  friend class CurveGenerator;
  friend class CurveRenderer;

private:
  /**
   * Private constructor, only the Generator can instatiate this class
   */
  Curve();

public:
  Curve (const Curve& c);

  virtual ~Curve();

  /**
   * returns false, iff no combination  resulted in a feasible path
   */
  bool is_valid();

  /**
   * Get the weight of the curve
   */
  double weight();



  /**
   * Start iterating over the points on this curve
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

  Pose2d goal() {return m_goal;}

  void set_trace(int value) {
    m_trace = value;
  }


private:
  void test_sequence(std::vector< CurveSegment* > &sequence);

  double handle_sequence(CurveSegment *segment1,
                         CurveSegment *segment2,
                         CurveSegment *segment3);

  double handle_sequence(CurveSegment *segment1,
                         CurveSegment *segment2,
                         CurveSegment *segment3,
                         CurveSegment *segment4);

  double handle_sequence(CircleSegment *circle1,
                         LineSegment *line,
                         CircleSegment *circle2);

  double handle_sequence(CircleSegment *circle1,
                         CircleSegment *line,
                         CircleSegment *circle2);

  double handle_sequence(CircleSegment *circle1,
                         CircleSegment *circle2,
                         CircleSegment *circle3,
                         CircleSegment *circle4);

  void init_circle_pairs(Pose2d &next_to, circle_pair &target);

  bool m_init;
  bool m_ignore_obstacles;

  float m_circle_radius;
  float m_max_waypoint_distance;

  bool m_use_map_cost;
  uint8_t m_min_cell_cost;
  float m_cost_forwards;
  float m_cost_backwards;
  float m_cost_curve;
  float m_cost_straight;

  std::vector<CurveSegment*> m_min_combo;
  float m_min_length;

  circle_pair m_circle_start;
  circle_pair m_circle_goal;

  Pose2d m_start;
  Pose2d m_goal;

  GridMap2d *m_map;

  bool m_iterating;
  unsigned m_output_number;

  int m_trace;
};

}

#endif /* CURVE_H_ */
