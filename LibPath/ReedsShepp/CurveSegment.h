/*
 * CurveSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CURVESEGMENT_H
#define CURVESEGMENT_H

/// PROJECT
#include "../common/GridMap2d.h"
#include "../common/Point2d.h"

/// SYSTEM
#include <assert.h>

#define NOT_FREE 999999

namespace lib_path
{

/**
 * The CurveSegment class is the base class for all segments a curve can constist of.
 */
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
     * Creates a new segment that is an exact copy of this one
     *
     * @return copy of this segment
     */
    virtual CurveSegment* clone() const = 0;

    /**
     * Setter for the MapInfo object
     *
     * @param map the new MapInfo
     */
    virtual void set_map(GridMap2d* map);

    /**
     * Setter for the maximum distance between two waypoints
     *
     * @param distance the new maximum distance
     */
    virtual void set_max_distance(float distance);

    /**
     * @brief Use the the map cell values to compute the cost of a curve or not.
     * @param arg Use the cell values
     */
    virtual void set_use_map_cost(bool arg);

    /**
     * @brief Set minimum cost per map cell.
     * @param cost Minimum cost.
     */
    virtual void set_min_cell_cost(uint8_t cost);

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
    virtual void set_trace(int value);

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
    GridMap2d* m_map;
    DIRECTION m_direction;
    float m_max_distance;

    /// Use the map cell values to calculate the cost of a curve?
    bool m_use_map_cost;

    /// Minimum cost of one cell
    uint8_t m_min_cell_cost;

    float m_cost_forwards;
    float m_cost_backwards;
    float m_cost_curve;
    float m_cost_straight;

    int m_trace;
};

}

#endif // CURVESEGMENT_H
