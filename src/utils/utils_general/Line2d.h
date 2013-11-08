/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2011

 @file Line2d.h
 */

#ifndef LINE2D_H
#define LINE2D_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Eigen
#include <Eigen/Core>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

/**
 * Represents a two dimensional straight line and offers several 2D line specific
 * methods.
 */
class Line2d {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Create a line through the given points (line direction is first point to
     * second point).
     * @attention The two points should differ from each other! Otherwise
     * origin and direction will be set to zero.
     *
     * @param p1 First point
     * @param p2 Second point
     */
    Line2d( const Eigen::Vector2d &p1, const Eigen::Vector2d &p2 );

    /**
     * Create a line with the given origin point and the given direction
     * angle.
     *
     * @param origin The line origin point.
     * @param angle Line direction angle.
     */
    Line2d( const Eigen::Vector2d &origin, const double angle );

    /**
     * Sets origin and direction to zero.
     */
    Line2d();

    /**
     * Empty.
     */
    virtual ~Line2d() {};

    /**
     * Reconfigure line to pass the given points (line direction is first to second point).
     * @attention The two points should differ from each other!
     *
     * @param p1 First point
     * @param p2 Second point
     *
     * @return True if everything is fine, false if the two point are equal.
     */
    bool Trough( const Eigen::Vector2d &p1, const Eigen::Vector2d &p2 );

    /**
     * Reconfigure the line to fullfill the equation y = a*x + b
     *
     * @param a Line slope.
     * @param b Line y-axis offset.
     */
    void FromPolynomial( double a, double b );

    /**
     * Set the line origin and the direction.
     *
     * @param origin The line origin point.
     * @param angle Line direction angle.
     */
    void FromAngle( const Eigen::Vector2d &origin, const double angle );

    /**
     * Returns the distance from the given point to the represented line.
     *
     * @return Distance to line.
     */
    double GetDistance( const Eigen::Vector2d &p ) const;

    double GetSignedDistance (const Eigen::Vector2d &p ) const;

    /**
     * Returns the relative location of the given point with respect to the line direction.
     *
     * @return negative value, positive value or 0 if the given point is located on the right,
     * left or on the represented line.
     */
    double GetRelativeLocation( const Eigen::Vector2d &p ) const;

    /**
     * Returns the direction vector of the represented line.
     *
     * @param vec Normalized vector parallel to line will be written to this parameter.
     */
    void GetDirection( Eigen::Vector2d &vec ) const { vec = mDirection; }

    /**
	 * Returns the direction vector of the represented line.
	 *
	 * @return Normalized vector parallel to line.
	 */
    Eigen::Vector2d GetDirection() const { return mDirection; }

    /**
     * Set the new line direction vector.
     *
     * @param dir The new direction vector.
     *
     * @return False if the given vector has length 0.
     */
    bool SetDirection( const Eigen::Vector2d &dir );

    /**
     * Set the direction vector.
     *
     * @param x x-value of the vector
     * @param y y-value of the vector
     *
     * @return False if both parameters are 0.
     */
    bool SetDirection( const double x, const double y );

    /**
     * Set the line origin.
     *
     * @param origin New origin.
     */
    void SetOrigin( const Eigen::Vector2d &origin ) { mOrigin = origin; }

    /**
     * Set the line origin.
     *
     * @param x x-value of the origin vector
     * @param y y-value of the origin vector
     */
    void SetOrigin( const double x = 0, const double y = 0 ) { mOrigin << x, y; }

    /**
     * Returns the origin point of the represented line. This is actual an
     * arbitrary choosen point that is located on the line.
     *
     * @paran vec The line origin will be written to this parameter.
     */
    void GetOrigin( Eigen::Vector2d &vec ) const { vec = mOrigin; }

    /**
	 * Returns the origin point of the represented line. This is actual an
	 * arbitrary choosen point that is located on the line.
	 *
	 * @return The line origin vector.
	 */
    Eigen::Vector2d GetOrigin() const { return mOrigin; }

    /**
	 * Change the direction about 180 degree (invert line direction).
	 */
	virtual void InvertDirection();

    /**
     * Return the angle of the represented line.
     *
     * @return Counter clockwise angle relative to the x-axis [rad]
     * in [-Pi, Pi]
     */
    double GetAngle() const;

    /**
     * Returns the angle between the two lines with respect to the line directions.
     *
     * @param line The other line.
     *
     * @return The angle (-Pi, Pi] [rad]
     */
    double GetAngle( const Line2d &line ) const;

    /**
     * Returns the projection of the given vector to the direction of this line.
     *
     * @param vec The vector.
     *
     * @return The projection with respect to the line direction, the result may be negative.
     * TODO Really?
     */
    double GetProjection( const Eigen::Vector2d &vec ) const;

    void ParallelShift( const double &dist );

    Eigen::Vector2d GetOrthoVector() const;

protected:
    /// Normalized vector parallel to line
    Eigen::Vector2d mDirection;

    /// Line origin
    Eigen::Vector2d mOrigin;
};

#endif // LINE2D_H
