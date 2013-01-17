/*
 * CircleSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CIRCLESEGMENT_H
#define CIRCLESEGMENT_H

#include "CurveSegment.h"

#include "../common/Point2d.h"

namespace lib_path
{

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
    CircleSegment(ORIENTATION orientation, DIRECTION direction);

    /**
     * Copy constructor
     */
    // not necessary, use default

    CurveSegment* clone() const;

    /**
     * Sets the center for this circle
     *
     * @param center point in map coordinates
     */
    void set_center_of_rotation(Point2d& center);

    /**
     * Sets the radius of the circle
     *
     * @param radius radius of the circle, NOT maximum steering angle!
     */
    void set_curve_radius(double radius);

    void set_start_angle_for_orientation(double orientation);
    void set_end_angle_for_orientation(double orientation);

    void set_start_angle(double angle);
    void set_end_angle(double angle);

    double get_start_angle() const;
    double get_end_angle() const;

    Point2d get_start_point() const;
    Point2d get_end_point() const;

    /**
     * Getter for the orientation
     */
    ORIENTATION orientation();

    bool has_orientation(ORIENTATION orientation);

    /**
     * Getter for the center
     */
    Point2d get_center() const;

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
    virtual float weight(bool ignore_obstacles);

private:
    void compute_arc_length();

    // midpoint circle
    bool checkCircle(int cx, int cy, int radius, bool ignore_obstacles);
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
