/*
 * CurveSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CURVESEGMENT_H
#define CURVESEGMENT_H

#include "../common/GridMap2d.h"
#include "../common/Point2d.h"

#include <assert.h>

#define NOT_FREE 999999

namespace lib_path {

class CurveSegment
{
public:
  enum DIRECTION {
    FORWARD,
    BACKWARD
  };

public:
  /**
   * Constructor, that sets the  direction
   *
   * @param direction FORWARD or BACKWARD
   */
  CurveSegment(DIRECTION direction);

  /**
   * Setter for the MapInfo object
   *
   * @param map the new MapInfo
   */
  virtual void set_map(GridMap2d *map);


  /**
   * Setter for the maximum distance between two waypoints
   *
   * @param distance the new maximum distance
   */
  virtual void set_max_distance(float distance);


  /**
   * Setter for the costs
   *
   * @param cost
   */
  virtual void set_cost_backwards(double cost);
  virtual void set_cost_forwards(double cost);
  virtual void set_cost_curve(double cost);
  virtual void set_cost_straight(double cost);

  /**
   * Debugging function:
   *   set the mapvalue to this value, where a "free check" has happened
   */
  virtual void set_trace(int value){
    m_trace = value;
  }


  /**
   * Getter for the direction
   */
  DIRECTION direction() const;



  /**
   * Start iterating over the points on this segment
   */
  virtual void reset_iteration() = 0;

  /**
   * Check, if another point exists
   *
   * @return true, iff there exists another point
   */
  virtual bool has_next() = 0;

  /**
   * Get the next point in this iteration
   */
  virtual Pose2d next() = 0;



  /**
   * Computes the weight of this segment
   */
  virtual float weight(bool ignore_obstacles) = 0;

protected:
  GridMap2d * m_map;
  DIRECTION m_direction;
  float m_max_distance;

  float m_cost_forwards;
  float m_cost_backwards;
  float m_cost_curve;
  float m_cost_straight;

  int m_trace;

};

}

#endif // CURVESEGMENT_H
