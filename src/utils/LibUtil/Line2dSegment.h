/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2011

 @file Line2dSegment.h
 */

#ifndef LINE2DSEGMENT_H_
#define LINE2DSEGMENT_H_

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <list>

// Eigen
#include <Eigen/Core>
//#include <Eigen/StdList>

// Project
#include "Line2d.h"

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

/**
 * Represents a straight line with finite length. Basically we add a length
 * variable to a straight line and offer some specific methods.
 */
class Line2dSegment : public Line2d {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Create a line segment with direction, origin  and length equal to zero.
	 */
	Line2dSegment();

	/**
	 * Empty.
	 */
	virtual ~Line2dSegment() {};

	/**
	 * Set start and end vector.
	 * @attention This may change the line direction an origin! The new line direction
	 * will be start to end, the line origin will be equalt to start.
	 *
	 * @param start Start vector.
	 * @param end End vector.
	 *
	 * @return False if start and end are equal.
	 */
	bool SetStartEnd( const Eigen::Vector2d &start, const Eigen::Vector2d &end );

	/**
	 * Returns the start vector (that is equal to the line origin).
	 *
	 * @param start The start vector will be copied to this parameter.
	 */
	void GetStart( Eigen::Vector2d &start ) const { start = mOrigin; }

	/**
	 * Returns the start vector (that is equal to the line origin).
	 *
	 * @return The start vector.
	 */
	Eigen::Vector2d GetStart() const { return mOrigin; }

	/**
	 * Returns the end vector.
	 *
	 * @param end The end vector will be copied to this parameter.
	 */
	void GetEnd( Eigen::Vector2d &end ) const { end = mOrigin + mLength * mDirection; }

	/**
	 * Returns the end vector.
	 *
	 * @return The end vector.
	 */
	Eigen::Vector2d GetEnd() const { return mOrigin + mLength * mDirection; }

	/**
	 * Returns the length of the line segment.
	 *
	 * @return Length [m].
	 */
	double GetLength() const { return mLength; }
	double GetMidDistance( const Line2d &line );

	double GetMinDistance( const Eigen::Vector2d &p ) const;

	void GetProjectedDistance( const Line2d &line, double &start, double &end ) const;

	virtual void InvertDirection();

private:
	/// Segment length
	double mLength;
};

#endif /* LINE2DSEGMENT_H_ */
