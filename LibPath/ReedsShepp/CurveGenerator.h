/*
 * CurveGenerator.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CURVEGENERATOR_H
#define CURVEGENERATOR_H

#include "Curve.h"
#include "CurveSegment.h"
#include "../common/GridMap2d.h"

#include <vector>
#include <string>
#include <ostream>

namespace lib_path {

class CurveGenerator
{
public:
  CurveGenerator();

  /**
   * Compiles a description of a Reeds-Shepp-Curve
   *
   * @param sequence string of segments
   *                   possible characters:
   *                   L - left curve
   *                   R - right curve
   *                   S - straight
   *                   | - change driving direction (forward <-> backward)
   * @returns true, iff the sequence was specified correctly
   */
  bool parse(std::string sequence, std::ostream& out=std::cerr);

  /**
   * Uses all parsed sequences to find the shortest path in a map
   *
   * @param start pose to start from in map coordinates
   * @param goal pose to end at in map coordinates
   * @param map MapInfo that is used to check for obstacles
   * @param curve_radius radius of circle elements, NOT the maximal steering angle
   * @param max_distance_between_waypoints Maximum distance between two connected waypoints
   * @param ignore_obstacles true <-> Find the shortest path, <b>ignoring</b> obstacles (default: false)
   *
   * @returns a Reeds-Shepp-Curve that describes the shortest path
   */
  Curve * find_path (const Pose2d &start, const Pose2d &goal, GridMap2d *map,
                     bool ignore_obstacles = false);

  /**
   * Setters
   */
  void set_circle_radius(double circle_radius);
  void set_max_waypoint_distance(double max_waypoint_distance);

  void set_cost_forwards(double cost_forwards);
  void set_cost_backwards(double cost_backwards);
  void set_cost_curve(double cost_curve);
  void set_cost_straight(double cost_straight);

  void set_trace(int value) {
    m_trace = value;
  }

private:
  std::vector< CurveSegment* > m_sequences[64];
  int m_count;

  double m_circle_radius;
  double m_max_waypoint_distance;
  double m_cost_forwards;
  double m_cost_backwards;
  double m_cost_curve;
  double m_cost_straight;

  int m_trace;
};

}

#endif // CURVEGENERATOR_H
