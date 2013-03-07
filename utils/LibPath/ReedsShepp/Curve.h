/*
 * Curve.h
 *
 *  Created on: Apr 2, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CURVE_H_
#define CURVE_H_

/// COMPONENT
#include "GeometryHelper.h"

/// SYSTEM
#include <vector>

namespace lib_path
{

class circle_pair
{
public:
    Point2d center_left;
    Point2d center_right;
};

/**
 * The Curve class represents a Reeds-Shepp Curve consisting of a sequence of circle and line shapes
 */
class Curve
{
    friend class CurveGenerator;
    friend class CurveRenderer;

private:
    /**
     * Private constructor, only the Generator can instatiate this class
     */
    Curve();

public:
    /**
     * Constructor
     */
    Curve(const Curve& c);

    /**
     * Destructor
     */
    virtual ~Curve();

    /**
     * returns false, iff no combination  resulted in a feasible path
     */
    bool is_valid();

    /**
     * Get the weight of the curve
     */
    double weight() const;

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

    /// ACCESSORS
    Pose2d start();
    Pose2d goal();

private:
    void set_trace(int value);
    int count();
    const CurveSegment& get_segment(int index);
    void add(const CurveSegment& segment);

private:
    double check_if_admissible();
    void init_segments();

    bool handle_sequence();
    double handle_sequence_CSC();
    double handle_sequence_C3();
    double handle_sequence_C4();

    void init_circle_pairs(Pose2d& next_to, circle_pair& target);

    bool m_ignore_obstacles;

    float m_circle_radius;
    float m_max_waypoint_distance;

    bool m_use_map_cost;
    uint8_t m_min_cell_cost;
    float m_cost_forwards;
    float m_cost_backwards;
    float m_cost_curve;
    float m_cost_straight;

    std::vector<CurveSegment*> m_sequence;
    double m_weight;

    circle_pair m_circle_start;
    circle_pair m_circle_goal;

    Pose2d m_start;
    Pose2d m_goal;

    GridMap2d* m_map;

    bool m_iterating;
    unsigned m_output_number;

    int m_trace;
};

}

#endif /* CURVE_H_ */
