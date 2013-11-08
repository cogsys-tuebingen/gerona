/*
 * CircleSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CIRCLESEGMENT_H
#define CIRCLESEGMENT_H

/// COMPONENT
#include "CurveSegment.h"

/// PROJECT
#include "../common/Point2d.h"

namespace lib_path
{

/**
 * The CircleSegment class represents a circle arc that has a direction
 */
class CircleSegment: public CurveSegment
{
    friend class GeometryHelper;
    friend class CurveRenderer;

public:
    enum ORIENTATION {
        LEFT_CURVE,
        RIGHT_CURVE
    };

public:
    /**
     * Constructor, that sets the orientation and the direction
     *
     * @param orientation LEFT or RIGHT
     * @param direction FORWARD or BACKWARD
     */
    CircleSegment(ORIENTATION get_orientation, DIRECTION direction);

    /**
     * Copy constructor
     */
    // not necessary, use default

    /**
     * Creates a new segment that is an exact copy of this one
     *
     * @return copy of this segment
     */
    CurveSegment* clone() const;

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
    virtual float weight(bool ignore_obstacles);


    /// SETTERS
    void set_center(Point2d& center);
    void set_radius(double get_radius);

    void set_start_angle_for_orientation(double orientation);
    void set_end_angle_for_orientation(double orientation);

    void set_start_angle(double angle);
    void set_end_angle(double angle);

    /// GETTERS
    double get_start_angle() const;
    double get_end_angle() const;

    Point2d get_start_point() const;
    Point2d get_end_point() const;

    ORIENTATION get_orientation() const;

    Point2d get_center() const;

    double get_radius() const;

private:
    void compute_arc_length();

    // midpoint circle
    bool checkCircle(int cx, int cy, int get_radius, bool ignore_obstacles);
    bool test8Points(int cx, int cy, int x, int y, bool ignore_obstacles);
    bool test4Points(int cx, int cy, int x, int y, bool ignore_obstacles);
    bool testPixel(int cx, int cy, int x, int y, bool ignore_obstacles);

    ORIENTATION m_orientation;

    Point2d m_center;
    double m_radius;

    double m_angle_start;
    double m_angle_end;

    double m_arc_length;
    double m_angle_step_size;

    bool m_iterating;
    int m_output;
    int m_steps;

    double m_weight;
};

}

#endif // CIRCLESEGMENT_H
